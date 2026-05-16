using System;
using System.Collections.Generic;
using System.Linq;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;
using Rhino;
using Rhino.Geometry;

namespace Spatial_Additive_Manufacturing
{
    public class ClassifiedMemberWeave_Component : GH_Component
    {
        private const double ConnectionTolerance = 0.01;
        private const int MaxHorizontalStartCandidates = 64;

        private sealed class ClassifiedSegment
        {
            public Line Line;
            public SMTMemberClassification Classification;
        }

        private sealed class VerticalAngledPathResult
        {
            public List<SMTClassifiedCurve> Paths { get; } = new List<SMTClassifiedCurve>();
            public List<SMTClassifiedCurve> LeftoverAngledMembers { get; } = new List<SMTClassifiedCurve>();
        }

        private sealed class HorizontalStartCandidate
        {
            public int Index { get; }
            public bool Forward { get; }

            public HorizontalStartCandidate(int index, bool forward)
            {
                Index = index;
                Forward = forward;
            }
        }

        public ClassifiedMemberWeave_Component()
          : base("Classified Member Weave", "Member Weave",
              "Weaves horizontal, vertical, and angled members into one sorted stream while preserving each member classification for SMT.",
              "FGAM", "Toolpathing")
        {
        }

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddTextParameter("Weave Pattern", "P", "Pattern tokens that define print order. Examples: H,V,A,V or a text list of Horizontal, Vertical, Angled.", GH_ParamAccess.list);
            pManager.AddLineParameter("Horizontal Members", "H", "Horizontal members from Continuous Toolpath Methods.", GH_ParamAccess.tree);
            pManager.AddLineParameter("Vertical Members", "V", "Vertical members from Continuous Toolpath Methods.", GH_ParamAccess.tree);
            pManager.AddLineParameter("Angled Members", "A", "Angled members from Continuous Toolpath Methods.", GH_ParamAccess.tree);

            pManager[0].Optional = true;
            pManager[1].Optional = true;
            pManager[2].Optional = true;
            pManager[3].Optional = true;
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("Sorted Classified Curves", "SC", "Sorted classified curves. Connect this to SMT Connection Classified's Sorted Classified Curves input.", GH_ParamAccess.list);
            pManager.AddCurveParameter("Preview Sorted Curves", "C", "Sorted member curves for preview only. Classification metadata is carried by SC.", GH_ParamAccess.list);
            pManager.AddTextParameter("Member Types", "T", "Classification for each sorted member.", GH_ParamAccess.list);
            pManager.AddTextParameter("Report", "R", "Summary of the weave operation.", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            var rawPattern = new List<string>();
            DA.GetDataList(0, rawPattern);
            bool hasExplicitPattern = rawPattern.Any(token => !string.IsNullOrWhiteSpace(token));

            var ignoredTokens = new List<string>();
            List<SMTMemberClassification> pattern = ParsePattern(rawPattern, ignoredTokens);
            if (pattern.Count == 0)
            {
                pattern.Add(SMTMemberClassification.Horizontal);
                pattern.Add(SMTMemberClassification.Vertical);
                pattern.Add(SMTMemberClassification.AngledDown);
            }

            foreach (string ignoredToken in ignoredTokens)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, $"Ignored unknown weave token: {ignoredToken}");
            }

            var horizontalMembersList = new List<SMTClassifiedCurve>();
            var verticalMembersList = new List<SMTClassifiedCurve>();
            var angledMembersList = new List<SMTClassifiedCurve>();

            if (DA.GetDataTree(1, out GH_Structure<GH_Line> horizontalMembers))
            {
                AddLinesToList(horizontalMembersList, horizontalMembers, SMTMemberClassification.Horizontal);
            }

            if (DA.GetDataTree(2, out GH_Structure<GH_Line> verticalMembers))
            {
                AddLinesToList(verticalMembersList, verticalMembers, SMTMemberClassification.Vertical);
            }

            if (DA.GetDataTree(3, out GH_Structure<GH_Line> angledMembers))
            {
                AddLinesToList(angledMembersList, angledMembers, SMTMemberClassification.AngledDown);
            }

            int horizontalCount = horizontalMembersList.Count;
            int verticalCount = verticalMembersList.Count;
            int angledCount = angledMembersList.Count;
            int inputCount = horizontalCount + verticalCount + angledCount;

            if (inputCount == 0)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "No member lines were supplied.");
                return;
            }

            var notices = new List<string>();
            List<SMTClassifiedCurve> wovenMembers;

            if (hasExplicitPattern)
            {
                Dictionary<SMTMemberClassification, Queue<SMTClassifiedCurve>> buckets = CreateEmptyBuckets();
                EnqueueMembers(buckets[SMTMemberClassification.Horizontal], horizontalMembersList);
                EnqueueMembers(buckets[SMTMemberClassification.Vertical], verticalMembersList);
                EnqueueMembers(buckets[SMTMemberClassification.AngledDown], angledMembersList);
                wovenMembers = WeaveMembers(buckets, pattern, notices);
                notices.Add("Used explicit weave pattern input for member sequencing.");
            }
            else
            {
                VerticalAngledPathResult verticalAngledPaths = BuildVerticalAngledPaths(
                    verticalMembersList,
                    angledMembersList,
                    notices);

                var verticalAngledMembers = new List<SMTClassifiedCurve>();
                verticalAngledMembers.AddRange(verticalAngledPaths.Paths);

                wovenMembers = WeaveConnectedMembers(
                    horizontalMembersList,
                    verticalAngledMembers,
                    notices);
            }

            List<SMTClassifiedCurve> continuousPaths = JoinAdjacentWovenMembers(wovenMembers, notices);

            DA.SetDataList(0, continuousPaths.Select(member => new GH_ObjectWrapper(member)));
            DA.SetDataList(1, continuousPaths.Select(member => member.Curve));
            DA.SetDataList(2, continuousPaths.Select(member => member.ToString()));
            DA.SetData(3, BuildReport(pattern, horizontalCount, verticalCount, angledCount, continuousPaths.Count, notices));
        }

        private static Dictionary<SMTMemberClassification, Queue<SMTClassifiedCurve>> CreateEmptyBuckets()
        {
            return new Dictionary<SMTMemberClassification, Queue<SMTClassifiedCurve>>
            {
                { SMTMemberClassification.Horizontal, new Queue<SMTClassifiedCurve>() },
                { SMTMemberClassification.Vertical, new Queue<SMTClassifiedCurve>() },
                { SMTMemberClassification.AngledDown, new Queue<SMTClassifiedCurve>() }
            };
        }

        private static void AddLinesToList(
            List<SMTClassifiedCurve> members,
            GH_Structure<GH_Line> lineTree,
            SMTMemberClassification classification)
        {
            foreach (GH_Line lineGoo in lineTree.AllData(true))
            {
                if (lineGoo == null || !lineGoo.IsValid)
                {
                    continue;
                }

                Line line = lineGoo.Value;
                if (!line.IsValid || line.Length <= RhinoMath.ZeroTolerance)
                {
                    continue;
                }

                if (classification == SMTMemberClassification.Vertical && line.From.Z > line.To.Z)
                {
                    line.Flip();
                }

                if (classification == SMTMemberClassification.AngledDown && line.From.Z < line.To.Z)
                {
                    line.Flip();
                }

                members.Add(new SMTClassifiedCurve(new LineCurve(line), classification));
            }
        }

        private static void EnqueueMembers(Queue<SMTClassifiedCurve> queue, IEnumerable<SMTClassifiedCurve> members)
        {
            foreach (SMTClassifiedCurve member in members)
            {
                queue.Enqueue(member);
            }
        }

        private static List<SMTMemberClassification> ParsePattern(IEnumerable<string> rawPattern, List<string> ignoredTokens)
        {
            var pattern = new List<SMTMemberClassification>();
            foreach (string rawTokenGroup in rawPattern)
            {
                foreach (string token in ExpandPatternTokens(rawTokenGroup))
                {
                    if (TryParsePatternToken(token, out SMTMemberClassification classification))
                    {
                        pattern.Add(classification);
                    }
                    else
                    {
                        ignoredTokens.Add(token);
                    }
                }
            }

            return pattern;
        }

        private static IEnumerable<string> ExpandPatternTokens(string rawTokenGroup)
        {
            if (string.IsNullOrWhiteSpace(rawTokenGroup))
            {
                yield break;
            }

            char[] separators = { ',', ';', '|', '/', '\\', ' ', '\t', '\r', '\n', '-', '>' };
            string[] tokens = rawTokenGroup.Split(separators, StringSplitOptions.RemoveEmptyEntries);
            if (tokens.Length == 1 && IsCompactPattern(tokens[0]))
            {
                foreach (char tokenCharacter in tokens[0])
                {
                    yield return tokenCharacter.ToString();
                }

                yield break;
            }

            foreach (string token in tokens)
            {
                yield return token;
            }
        }

        private static bool IsCompactPattern(string token)
        {
            return token.All(character =>
                character == 'H' || character == 'h' ||
                character == 'V' || character == 'v' ||
                character == 'A' || character == 'a');
        }

        private static bool TryParsePatternToken(string token, out SMTMemberClassification classification)
        {
            classification = SMTMemberClassification.Geometric;
            string normalized = token.Trim().ToUpperInvariant();

            if (normalized == "H" || normalized == "HOR" || normalized == "HORIZ" || normalized == "HORIZONTAL")
            {
                classification = SMTMemberClassification.Horizontal;
                return true;
            }

            if (normalized == "V" || normalized == "VERT" || normalized == "VERTICAL" || normalized == "STRUT")
            {
                classification = SMTMemberClassification.Vertical;
                return true;
            }

            if (normalized == "A" || normalized == "ANG" || normalized == "ANGLE" || normalized == "ANGLED" || normalized == "D" || normalized == "DIAGONAL" || normalized == "BRACE")
            {
                classification = SMTMemberClassification.AngledDown;
                return true;
            }

            return false;
        }

        private static List<SMTClassifiedCurve> WeaveConnectedMembers(
            List<SMTClassifiedCurve> horizontalMembers,
            List<SMTClassifiedCurve> verticalAngledMembers,
            List<string> notices)
        {
            var wovenMembers = new List<SMTClassifiedCurve>();
            var remainingHorizontals = new List<SMTClassifiedCurve>(horizontalMembers);
            var remainingVerticalAngled = new List<SMTClassifiedCurve>(verticalAngledMembers);
            var frontier = new List<Point3d>();
            int horizontalGroupCount = 0;
            int connectedVerticalAngledCount = 0;
            int orphanVerticalAngledCount = 0;

            while (remainingHorizontals.Count > 0 || remainingVerticalAngled.Count > 0)
            {
                int horizontalIndex = FindNextHorizontalIndex(remainingHorizontals, frontier);
                if (horizontalIndex >= 0)
                {
                    List<int> horizontalGroup = CollectConnectedHorizontalGroup(remainingHorizontals, horizontalIndex);
                    List<SMTClassifiedCurve> orderedGroup = OrderHorizontalGroup(remainingHorizontals, horizontalGroup, frontier);
                    RemoveMembersAt(remainingHorizontals, horizontalGroup);

                    wovenMembers.AddRange(orderedGroup);
                    horizontalGroupCount++;

                    frontier = GetEndpointPoints(orderedGroup);

                    List<int> connectedVerticalAngled = CollectVerticalAngledStartingAtFrontier(
                        remainingVerticalAngled,
                        frontier);

                    if (connectedVerticalAngled.Count == 0)
                    {
                        frontier.Clear();
                        continue;
                    }

                    List<SMTClassifiedCurve> connectedMembers = connectedVerticalAngled
                        .Select(index => remainingVerticalAngled[index])
                        .ToList();
                    RemoveMembersAt(remainingVerticalAngled, connectedVerticalAngled);

                    var nextFrontier = new List<Point3d>();
                    foreach (SMTClassifiedCurve member in connectedMembers)
                    {
                        wovenMembers.Add(member);
                        AddVerticalAngledExitPoints(member, frontier, nextFrontier);
                        connectedVerticalAngledCount++;
                    }

                    frontier = nextFrontier;
                    continue;
                }

                int verticalAngledIndex = FindNextVerticalAngledIndex(remainingVerticalAngled, frontier);
                if (verticalAngledIndex < 0)
                {
                    break;
                }

                SMTClassifiedCurve orphanMember = remainingVerticalAngled[verticalAngledIndex];
                remainingVerticalAngled.RemoveAt(verticalAngledIndex);
                wovenMembers.Add(orphanMember);
                frontier = GetEndpointPoints(orphanMember);
                orphanVerticalAngledCount++;
            }

            if (horizontalGroupCount > 0)
            {
                notices.Add($"Sequenced {horizontalGroupCount} connected horizontal group(s) before their attached vertical/angled path(s).");
            }

            if (connectedVerticalAngledCount > 0)
            {
                notices.Add($"Placed {connectedVerticalAngledCount} vertical/angled path(s) directly after the horizontal group they start from.");
            }

            if (orphanVerticalAngledCount > 0)
            {
                notices.Add($"{orphanVerticalAngledCount} vertical/angled path(s) had no unused horizontal start connection and were appended by proximity.");
            }

            return wovenMembers;
        }

        private static int FindNextHorizontalIndex(
            List<SMTClassifiedCurve> horizontalMembers,
            List<Point3d> frontier)
        {
            if (frontier.Count > 0)
            {
                for (int i = 0; i < horizontalMembers.Count; i++)
                {
                    if (MemberTouchesAnyPoint(horizontalMembers[i], frontier))
                    {
                        return i;
                    }
                }
            }

            return FindLowestMemberIndex(horizontalMembers);
        }

        private static int FindNextVerticalAngledIndex(
            List<SMTClassifiedCurve> verticalAngledMembers,
            List<Point3d> frontier)
        {
            if (frontier.Count > 0)
            {
                for (int i = 0; i < verticalAngledMembers.Count; i++)
                {
                    if (MemberStartTouchesAnyPoint(verticalAngledMembers[i], frontier))
                    {
                        return i;
                    }
                }
            }

            return FindLowestMemberIndex(verticalAngledMembers);
        }

        private static int FindLowestMemberIndex(List<SMTClassifiedCurve> members)
        {
            int bestIndex = -1;
            double bestZ = double.MaxValue;
            double bestX = double.MaxValue;
            double bestY = double.MaxValue;

            for (int i = 0; i < members.Count; i++)
            {
                Point3d center = MemberCenter(members[i]);
                double z = MinMemberZ(members[i]);
                if (z < bestZ - ConnectionTolerance ||
                    (Math.Abs(z - bestZ) <= ConnectionTolerance && center.X < bestX - ConnectionTolerance) ||
                    (Math.Abs(z - bestZ) <= ConnectionTolerance && Math.Abs(center.X - bestX) <= ConnectionTolerance && center.Y < bestY))
                {
                    bestIndex = i;
                    bestZ = z;
                    bestX = center.X;
                    bestY = center.Y;
                }
            }

            return bestIndex;
        }

        private static List<int> CollectConnectedHorizontalGroup(
            List<SMTClassifiedCurve> horizontalMembers,
            int startIndex)
        {
            var group = new List<int>();
            var queued = new HashSet<int> { startIndex };
            var queue = new Queue<int>();
            queue.Enqueue(startIndex);

            while (queue.Count > 0)
            {
                int currentIndex = queue.Dequeue();
                group.Add(currentIndex);

                for (int i = 0; i < horizontalMembers.Count; i++)
                {
                    if (queued.Contains(i))
                    {
                        continue;
                    }

                    if (MembersShareEndpoint(horizontalMembers[currentIndex], horizontalMembers[i]))
                    {
                        queued.Add(i);
                        queue.Enqueue(i);
                    }
                }
            }

            return group;
        }

        private static List<SMTClassifiedCurve> OrderHorizontalGroup(
            List<SMTClassifiedCurve> horizontalMembers,
            List<int> horizontalGroup,
            List<Point3d> frontier)
        {
            var orderedMembers = new List<SMTClassifiedCurve>();
            var remaining = new HashSet<int>(horizontalGroup);

            while (remaining.Count > 0)
            {
                List<SMTClassifiedCurve> chain = ExtractLongestHorizontalChain(
                    horizontalMembers,
                    remaining,
                    frontier);

                if (chain.Count == 0)
                {
                    int fallbackIndex = FindLowestIndexInSet(horizontalMembers, remaining);
                    if (fallbackIndex < 0)
                    {
                        break;
                    }

                    chain.Add(horizontalMembers[fallbackIndex]);
                    remaining.Remove(fallbackIndex);
                }

                orderedMembers.AddRange(chain);

                // Only the first chain should be biased toward the incoming frontier.
                frontier = new List<Point3d>();
            }

            return orderedMembers;
        }

        private static List<SMTClassifiedCurve> ExtractLongestHorizontalChain(
            List<SMTClassifiedCurve> horizontalMembers,
            HashSet<int> remaining,
            List<Point3d> frontier)
        {
            List<HorizontalStartCandidate> candidates = CollectHorizontalStartCandidates(horizontalMembers, remaining, frontier);
            var bestChain = new List<SMTClassifiedCurve>();
            var bestUsedIndices = new List<int>();
            int bestStartIndex = -1;
            double bestLength = -1.0;

            foreach (HorizontalStartCandidate candidate in candidates)
            {
                List<SMTClassifiedCurve> chain = BuildHorizontalChainCandidate(
                    horizontalMembers,
                    remaining,
                    candidate,
                    out List<int> usedIndices);

                double chainLength = chain.Sum(MemberLength);
                bool isBetter =
                    chain.Count > bestChain.Count ||
                    (chain.Count == bestChain.Count && chainLength > bestLength + ConnectionTolerance) ||
                    (chain.Count == bestChain.Count &&
                     Math.Abs(chainLength - bestLength) <= ConnectionTolerance &&
                     IsMemberIndexBefore(horizontalMembers, candidate.Index, bestStartIndex));

                if (isBetter)
                {
                    bestChain = chain;
                    bestUsedIndices = usedIndices;
                    bestStartIndex = candidate.Index;
                    bestLength = chainLength;
                }
            }

            foreach (int index in bestUsedIndices)
            {
                remaining.Remove(index);
            }

            return bestChain;
        }

        private static List<HorizontalStartCandidate> CollectHorizontalStartCandidates(
            List<SMTClassifiedCurve> horizontalMembers,
            HashSet<int> remaining,
            List<Point3d> frontier)
        {
            var candidates = new List<HorizontalStartCandidate>();
            var seen = new HashSet<string>();
            List<int> orderedIndices = remaining
                .OrderBy(index => MinMemberZ(horizontalMembers[index]))
                .ThenBy(index => MemberCenter(horizontalMembers[index]).X)
                .ThenBy(index => MemberCenter(horizontalMembers[index]).Y)
                .ToList();

            if (frontier.Count > 0)
            {
                foreach (int index in orderedIndices)
                {
                    if (!TryGetMemberEndpoints(horizontalMembers[index], out Point3d start, out Point3d end))
                    {
                        continue;
                    }

                    if (PointTouchesAny(start, frontier))
                    {
                        AddHorizontalStartCandidate(candidates, seen, index, true);
                    }

                    if (PointTouchesAny(end, frontier))
                    {
                        AddHorizontalStartCandidate(candidates, seen, index, false);
                    }
                }

                if (candidates.Count > 0)
                {
                    return LimitHorizontalStartCandidates(horizontalMembers, candidates);
                }
            }

            foreach (int index in orderedIndices)
            {
                if (!TryGetMemberEndpoints(horizontalMembers[index], out Point3d start, out Point3d end))
                {
                    continue;
                }

                int startDegree = CountHorizontalEdgesTouchingPoint(horizontalMembers, remaining, start);
                int endDegree = CountHorizontalEdgesTouchingPoint(horizontalMembers, remaining, end);

                if (startDegree != 2)
                {
                    AddHorizontalStartCandidate(candidates, seen, index, true);
                }

                if (endDegree != 2)
                {
                    AddHorizontalStartCandidate(candidates, seen, index, false);
                }
            }

            if (candidates.Count == 0)
            {
                int fallbackIndex = FindLowestIndexInSet(horizontalMembers, remaining);
                if (fallbackIndex >= 0 &&
                    TryGetMemberEndpoints(horizontalMembers[fallbackIndex], out Point3d start, out Point3d end))
                {
                    bool lowerEndpointFirst = ComparePointsForTraversal(start, end) <= 0;
                    AddHorizontalStartCandidate(candidates, seen, fallbackIndex, lowerEndpointFirst);
                    AddHorizontalStartCandidate(candidates, seen, fallbackIndex, !lowerEndpointFirst);
                }
            }

            return LimitHorizontalStartCandidates(horizontalMembers, candidates);
        }

        private static List<HorizontalStartCandidate> LimitHorizontalStartCandidates(
            List<SMTClassifiedCurve> horizontalMembers,
            List<HorizontalStartCandidate> candidates)
        {
            return candidates
                .OrderBy(candidate => MinMemberZ(horizontalMembers[candidate.Index]))
                .ThenBy(candidate => MemberCenter(horizontalMembers[candidate.Index]).X)
                .ThenBy(candidate => MemberCenter(horizontalMembers[candidate.Index]).Y)
                .ThenBy(candidate => candidate.Forward ? 0 : 1)
                .Take(MaxHorizontalStartCandidates)
                .ToList();
        }

        private static void AddHorizontalStartCandidate(
            List<HorizontalStartCandidate> candidates,
            HashSet<string> seen,
            int index,
            bool forward)
        {
            string key = $"{index}:{forward}";
            if (seen.Add(key))
            {
                candidates.Add(new HorizontalStartCandidate(index, forward));
            }
        }

        private static List<SMTClassifiedCurve> BuildHorizontalChainCandidate(
            List<SMTClassifiedCurve> horizontalMembers,
            HashSet<int> remaining,
            HorizontalStartCandidate startCandidate,
            out List<int> usedIndices)
        {
            var available = new HashSet<int>(remaining);
            var chain = new List<SMTClassifiedCurve>();
            usedIndices = new List<int>();
            int currentIndex = startCandidate.Index;
            bool forward = startCandidate.Forward;

            while (available.Contains(currentIndex))
            {
                SMTClassifiedCurve member = horizontalMembers[currentIndex];
                if (!TryGetMemberEndpoints(member, out Point3d start, out Point3d end))
                {
                    available.Remove(currentIndex);
                    break;
                }

                SMTClassifiedCurve orientedMember = forward ? member : ReverseClassifiedCurve(member);
                Point3d currentEnd = forward ? end : start;

                chain.Add(orientedMember);
                usedIndices.Add(currentIndex);
                available.Remove(currentIndex);

                currentIndex = FindNextHorizontalByFleury(
                    horizontalMembers,
                    available,
                    currentEnd,
                    out forward);

                if (currentIndex < 0)
                {
                    break;
                }
            }

            return chain;
        }

        private static int FindNextHorizontalByFleury(
            List<SMTClassifiedCurve> horizontalMembers,
            HashSet<int> available,
            Point3d currentPoint,
            out bool forward)
        {
            forward = true;
            var candidates = new List<HorizontalStartCandidate>();
            var seen = new HashSet<string>();

            foreach (int index in available)
            {
                if (!TryGetMemberEndpoints(horizontalMembers[index], out Point3d start, out Point3d end))
                {
                    continue;
                }

                if (PointsMatch(currentPoint, start))
                {
                    AddHorizontalStartCandidate(candidates, seen, index, true);
                }

                if (PointsMatch(currentPoint, end))
                {
                    AddHorizontalStartCandidate(candidates, seen, index, false);
                }
            }

            if (candidates.Count == 0)
            {
                return -1;
            }

            if (candidates.Count == 1)
            {
                forward = candidates[0].Forward;
                return candidates[0].Index;
            }

            List<HorizontalStartCandidate> nonBridgeCandidates = candidates
                .Where(candidate => !IsBridgeHorizontalChoice(horizontalMembers, available, currentPoint, candidate))
                .ToList();

            List<HorizontalStartCandidate> viableCandidates = nonBridgeCandidates.Count > 0
                ? nonBridgeCandidates
                : candidates;

            HorizontalStartCandidate bestCandidate = null;
            int bestFutureDegree = -1;

            foreach (HorizontalStartCandidate candidate in viableCandidates)
            {
                if (!TryGetHorizontalTraversalEndPoint(horizontalMembers, candidate, out Point3d nextPoint))
                {
                    continue;
                }

                int futureDegree = CountHorizontalEdgesTouchingPoint(horizontalMembers, available, nextPoint) - 1;
                if (bestCandidate == null ||
                    futureDegree > bestFutureDegree ||
                    (futureDegree == bestFutureDegree && IsMemberIndexBefore(horizontalMembers, candidate.Index, bestCandidate.Index)))
                {
                    bestCandidate = candidate;
                    bestFutureDegree = futureDegree;
                }
            }

            if (bestCandidate == null)
            {
                return -1;
            }

            forward = bestCandidate.Forward;
            return bestCandidate.Index;
        }

        private static bool IsBridgeHorizontalChoice(
            List<SMTClassifiedCurve> horizontalMembers,
            HashSet<int> available,
            Point3d currentPoint,
            HorizontalStartCandidate candidate)
        {
            int reachableBefore = CountReachableHorizontalEdges(horizontalMembers, available, currentPoint);
            if (reachableBefore <= 1)
            {
                return false;
            }

            var availableAfterChoice = new HashSet<int>(available);
            availableAfterChoice.Remove(candidate.Index);
            int reachableAfter = CountReachableHorizontalEdges(horizontalMembers, availableAfterChoice, currentPoint);

            return reachableAfter < reachableBefore - 1;
        }

        private static int CountReachableHorizontalEdges(
            List<SMTClassifiedCurve> horizontalMembers,
            HashSet<int> available,
            Point3d startPoint)
        {
            var visitedEdges = new HashSet<int>();
            var visitedPoints = new List<Point3d>();
            var queue = new Queue<Point3d>();
            AddUniquePoint(visitedPoints, startPoint);
            queue.Enqueue(startPoint);

            while (queue.Count > 0)
            {
                Point3d point = queue.Dequeue();
                foreach (int index in available)
                {
                    if (visitedEdges.Contains(index) ||
                        !TryGetMemberEndpoints(horizontalMembers[index], out Point3d start, out Point3d end))
                    {
                        continue;
                    }

                    if (!PointsMatch(point, start) && !PointsMatch(point, end))
                    {
                        continue;
                    }

                    visitedEdges.Add(index);
                    if (!visitedPoints.Any(existing => PointsMatch(existing, start)))
                    {
                        AddUniquePoint(visitedPoints, start);
                        queue.Enqueue(start);
                    }

                    if (!visitedPoints.Any(existing => PointsMatch(existing, end)))
                    {
                        AddUniquePoint(visitedPoints, end);
                        queue.Enqueue(end);
                    }
                }
            }

            return visitedEdges.Count;
        }

        private static int CountHorizontalEdgesTouchingPoint(
            List<SMTClassifiedCurve> horizontalMembers,
            HashSet<int> available,
            Point3d point)
        {
            int count = 0;
            foreach (int index in available)
            {
                if (!TryGetMemberEndpoints(horizontalMembers[index], out Point3d start, out Point3d end))
                {
                    continue;
                }

                if (PointsMatch(point, start) || PointsMatch(point, end))
                {
                    count++;
                }
            }

            return count;
        }

        private static bool TryGetHorizontalTraversalEndPoint(
            List<SMTClassifiedCurve> horizontalMembers,
            HorizontalStartCandidate candidate,
            out Point3d endPoint)
        {
            if (!TryGetMemberEndpoints(horizontalMembers[candidate.Index], out Point3d start, out Point3d end))
            {
                endPoint = Point3d.Unset;
                return false;
            }

            endPoint = candidate.Forward ? end : start;
            return true;
        }

        private static int FindHorizontalGroupStart(
            List<SMTClassifiedCurve> horizontalMembers,
            List<int> horizontalGroup,
            List<Point3d> frontier)
        {
            if (frontier.Count > 0)
            {
                foreach (int index in horizontalGroup)
                {
                    if (MemberTouchesAnyPoint(horizontalMembers[index], frontier))
                    {
                        return index;
                    }
                }
            }

            return FindLowestIndexInSet(horizontalMembers, new HashSet<int>(horizontalGroup));
        }

        private static int FindLowestIndexInSet(List<SMTClassifiedCurve> members, HashSet<int> candidateIndices)
        {
            int bestIndex = -1;
            double bestZ = double.MaxValue;
            double bestX = double.MaxValue;
            double bestY = double.MaxValue;

            foreach (int index in candidateIndices)
            {
                Point3d center = MemberCenter(members[index]);
                double z = MinMemberZ(members[index]);
                if (z < bestZ - ConnectionTolerance ||
                    (Math.Abs(z - bestZ) <= ConnectionTolerance && center.X < bestX - ConnectionTolerance) ||
                    (Math.Abs(z - bestZ) <= ConnectionTolerance && Math.Abs(center.X - bestX) <= ConnectionTolerance && center.Y < bestY))
                {
                    bestIndex = index;
                    bestZ = z;
                    bestX = center.X;
                    bestY = center.Y;
                }
            }

            return bestIndex;
        }

        private static int FindNextTouchingHorizontalIndex(
            List<SMTClassifiedCurve> horizontalMembers,
            HashSet<int> remaining,
            SMTClassifiedCurve currentMember,
            out bool reverseNext)
        {
            reverseNext = false;
            if (!TryGetMemberEndpoints(currentMember, out _, out Point3d currentEnd))
            {
                return -1;
            }

            foreach (int index in remaining)
            {
                if (!TryGetMemberEndpoints(horizontalMembers[index], out Point3d candidateStart, out Point3d candidateEnd))
                {
                    continue;
                }

                if (PointsMatch(currentEnd, candidateStart))
                {
                    return index;
                }

                if (PointsMatch(currentEnd, candidateEnd))
                {
                    reverseNext = true;
                    return index;
                }
            }

            return -1;
        }

        private static List<int> CollectVerticalAngledStartingAtFrontier(
            List<SMTClassifiedCurve> verticalAngledMembers,
            List<Point3d> frontier)
        {
            var connectedIndices = new List<int>();
            for (int i = 0; i < verticalAngledMembers.Count; i++)
            {
                if (MemberStartTouchesAnyPoint(verticalAngledMembers[i], frontier))
                {
                    connectedIndices.Add(i);
                }
            }

            return connectedIndices;
        }

        private static void RemoveMembersAt<T>(List<T> members, IEnumerable<int> indices)
        {
            foreach (int index in indices.Distinct().OrderByDescending(index => index))
            {
                if (index >= 0 && index < members.Count)
                {
                    members.RemoveAt(index);
                }
            }
        }

        private static void AddVerticalAngledExitPoints(
            SMTClassifiedCurve member,
            List<Point3d> entryFrontier,
            List<Point3d> exitFrontier)
        {
            if (!TryGetMemberEndpoints(member, out Point3d start, out Point3d end))
            {
                return;
            }

            if (PointTouchesAny(start, entryFrontier))
            {
                AddUniquePoint(exitFrontier, end);
                return;
            }

            if (PointTouchesAny(end, entryFrontier))
            {
                AddUniquePoint(exitFrontier, start);
                return;
            }

            AddUniquePoint(exitFrontier, start);
            AddUniquePoint(exitFrontier, end);
        }

        private static List<Point3d> GetEndpointPoints(IEnumerable<SMTClassifiedCurve> members)
        {
            var points = new List<Point3d>();
            foreach (SMTClassifiedCurve member in members)
            {
                AddEndpointPoints(member, points);
            }

            return points;
        }

        private static List<Point3d> GetEndpointPoints(SMTClassifiedCurve member)
        {
            var points = new List<Point3d>();
            AddEndpointPoints(member, points);
            return points;
        }

        private static void AddEndpointPoints(SMTClassifiedCurve member, List<Point3d> points)
        {
            if (!TryGetMemberEndpoints(member, out Point3d start, out Point3d end))
            {
                return;
            }

            AddUniquePoint(points, start);
            AddUniquePoint(points, end);
        }

        private static void AddUniquePoint(List<Point3d> points, Point3d point)
        {
            if (!points.Any(existing => PointsMatch(existing, point)))
            {
                points.Add(point);
            }
        }

        private static bool MemberTouchesAnyPoint(SMTClassifiedCurve member, List<Point3d> points)
        {
            if (!TryGetMemberEndpoints(member, out Point3d start, out Point3d end))
            {
                return false;
            }

            return PointTouchesAny(start, points) || PointTouchesAny(end, points);
        }

        private static bool MemberStartTouchesAnyPoint(SMTClassifiedCurve member, List<Point3d> points)
        {
            if (!TryGetMemberEndpoints(member, out Point3d start, out _))
            {
                return false;
            }

            return PointTouchesAny(start, points);
        }

        private static bool PointTouchesAny(Point3d point, List<Point3d> points)
        {
            return points.Any(candidate => PointsMatch(point, candidate));
        }

        private static bool MembersShareEndpoint(SMTClassifiedCurve first, SMTClassifiedCurve second)
        {
            if (!TryGetMemberEndpoints(first, out Point3d firstStart, out Point3d firstEnd) ||
                !TryGetMemberEndpoints(second, out Point3d secondStart, out Point3d secondEnd))
            {
                return false;
            }

            return PointsMatch(firstStart, secondStart) ||
                   PointsMatch(firstStart, secondEnd) ||
                   PointsMatch(firstEnd, secondStart) ||
                   PointsMatch(firstEnd, secondEnd);
        }

        private static bool TryGetMemberEndpoints(SMTClassifiedCurve member, out Point3d start, out Point3d end)
        {
            List<ClassifiedSegment> segments = ExtractSegments(member).ToList();
            if (segments.Count == 0)
            {
                start = Point3d.Unset;
                end = Point3d.Unset;
                return false;
            }

            start = segments[0].Line.From;
            end = segments[segments.Count - 1].Line.To;
            return true;
        }

        private static double MinMemberZ(SMTClassifiedCurve member)
        {
            List<ClassifiedSegment> segments = ExtractSegments(member).ToList();
            if (segments.Count == 0)
            {
                return double.MaxValue;
            }

            return segments.Min(segment => MinZ(segment.Line));
        }

        private static Point3d MemberCenter(SMTClassifiedCurve member)
        {
            List<ClassifiedSegment> segments = ExtractSegments(member).ToList();
            if (segments.Count == 0)
            {
                return Point3d.Origin;
            }

            double x = 0.0;
            double y = 0.0;
            double z = 0.0;
            int count = 0;
            foreach (ClassifiedSegment segment in segments)
            {
                x += segment.Line.From.X + segment.Line.To.X;
                y += segment.Line.From.Y + segment.Line.To.Y;
                z += segment.Line.From.Z + segment.Line.To.Z;
                count += 2;
            }

            return new Point3d(x / count, y / count, z / count);
        }

        private static double MemberLength(SMTClassifiedCurve member)
        {
            return ExtractSegments(member).Sum(segment => segment.Line.Length);
        }

        private static bool IsMemberIndexBefore(List<SMTClassifiedCurve> members, int candidateIndex, int currentBestIndex)
        {
            if (currentBestIndex < 0)
            {
                return true;
            }

            double candidateZ = MinMemberZ(members[candidateIndex]);
            double currentBestZ = MinMemberZ(members[currentBestIndex]);
            if (candidateZ < currentBestZ - ConnectionTolerance)
            {
                return true;
            }

            if (candidateZ > currentBestZ + ConnectionTolerance)
            {
                return false;
            }

            Point3d candidateCenter = MemberCenter(members[candidateIndex]);
            Point3d currentBestCenter = MemberCenter(members[currentBestIndex]);
            if (candidateCenter.X < currentBestCenter.X - ConnectionTolerance)
            {
                return true;
            }

            if (candidateCenter.X > currentBestCenter.X + ConnectionTolerance)
            {
                return false;
            }

            if (candidateCenter.Y < currentBestCenter.Y - ConnectionTolerance)
            {
                return true;
            }

            if (candidateCenter.Y > currentBestCenter.Y + ConnectionTolerance)
            {
                return false;
            }

            return candidateIndex < currentBestIndex;
        }

        private static int ComparePointsForTraversal(Point3d first, Point3d second)
        {
            if (first.Z < second.Z - ConnectionTolerance)
            {
                return -1;
            }

            if (first.Z > second.Z + ConnectionTolerance)
            {
                return 1;
            }

            if (first.X < second.X - ConnectionTolerance)
            {
                return -1;
            }

            if (first.X > second.X + ConnectionTolerance)
            {
                return 1;
            }

            if (first.Y < second.Y - ConnectionTolerance)
            {
                return -1;
            }

            if (first.Y > second.Y + ConnectionTolerance)
            {
                return 1;
            }

            return 0;
        }

        private static SMTClassifiedCurve ReverseClassifiedCurve(SMTClassifiedCurve member)
        {
            return ToClassifiedCurve(ReversePath(ExtractSegments(member).ToList()));
        }

        private static List<SMTClassifiedCurve> WeaveMembers(
            Dictionary<SMTMemberClassification, Queue<SMTClassifiedCurve>> buckets,
            List<SMTMemberClassification> pattern,
            List<string> notices)
        {
            var wovenMembers = new List<SMTClassifiedCurve>();

            while (RemainingCount(buckets) > 0)
            {
                bool addedInCycle = false;
                foreach (SMTMemberClassification classification in pattern)
                {
                    Queue<SMTClassifiedCurve> bucket = buckets[classification];
                    if (bucket.Count == 0)
                    {
                        continue;
                    }

                    wovenMembers.Add(bucket.Dequeue());
                    addedInCycle = true;
                }

                if (!addedInCycle)
                {
                    AppendRemainingMembers(buckets, wovenMembers, notices);
                    break;
                }
            }

            return wovenMembers;
        }

        private static int RemainingCount(Dictionary<SMTMemberClassification, Queue<SMTClassifiedCurve>> buckets)
        {
            return buckets[SMTMemberClassification.Horizontal].Count +
                   buckets[SMTMemberClassification.Vertical].Count +
                   buckets[SMTMemberClassification.AngledDown].Count;
        }

        private static void AppendRemainingMembers(
            Dictionary<SMTMemberClassification, Queue<SMTClassifiedCurve>> buckets,
            List<SMTClassifiedCurve> wovenMembers,
            List<string> notices)
        {
            foreach (SMTMemberClassification classification in new[]
            {
                SMTMemberClassification.Horizontal,
                SMTMemberClassification.Vertical,
                SMTMemberClassification.AngledDown
            })
            {
                int remaining = buckets[classification].Count;
                if (remaining == 0)
                {
                    continue;
                }

                notices.Add($"Pattern did not include available {classification} members; appended {remaining} at the end.");
                while (buckets[classification].Count > 0)
                {
                    wovenMembers.Add(buckets[classification].Dequeue());
                }
            }
        }

        private static VerticalAngledPathResult BuildVerticalAngledPaths(
            List<SMTClassifiedCurve> verticalMembers,
            List<SMTClassifiedCurve> angledMembers,
            List<string> notices)
        {
            var result = new VerticalAngledPathResult();
            List<ClassifiedSegment> verticals = verticalMembers.SelectMany(ExtractSegments).ToList();
            List<ClassifiedSegment> angledDowns = angledMembers.SelectMany(ExtractSegments).ToList();

            int chainCountWithAngles = 0;

            while (verticals.Count > 0)
            {
                int startVerticalIndex = FindStartVertical(verticals, angledDowns);
                if (startVerticalIndex < 0)
                {
                    break;
                }

                List<ClassifiedSegment> chain = BuildVerticalAngledChain(
                    startVerticalIndex,
                    verticals,
                    angledDowns);

                if (chain.Count > 0)
                {
                    if (chain.Any(segment => IsAngled(segment.Classification)))
                    {
                        chainCountWithAngles++;
                    }

                    result.Paths.Add(ToClassifiedCurve(chain));
                }
            }

            foreach (ClassifiedSegment angle in angledDowns)
            {
                result.LeftoverAngledMembers.Add(new SMTClassifiedCurve(new LineCurve(angle.Line), angle.Classification));
            }

            if (chainCountWithAngles > 0)
            {
                notices.Add($"Built {chainCountWithAngles} continuous Vertical -> AngledDown chain(s).");
            }

            if (result.LeftoverAngledMembers.Count > 0)
            {
                notices.Add($"{result.LeftoverAngledMembers.Count} angled member(s) were skipped because they were not attached to vertical members in the same layer.");
            }

            return result;
        }

        private static List<ClassifiedSegment> BuildVerticalAngledChain(
            int startVerticalIndex,
            List<ClassifiedSegment> verticals,
            List<ClassifiedSegment> angledDowns)
        {
            var chain = new List<ClassifiedSegment>();
            int currentVerticalIndex = startVerticalIndex;

            while (currentVerticalIndex >= 0 && currentVerticalIndex < verticals.Count)
            {
                ClassifiedSegment currentVertical = verticals[currentVerticalIndex];
                chain.Add(currentVertical);
                verticals.RemoveAt(currentVerticalIndex);

                int angleIndex = FindOutgoingAngleIndex(
                    currentVertical.Line.To,
                    verticals,
                    angledDowns,
                    out int nextVerticalIndex);

                if (angleIndex < 0 || nextVerticalIndex < 0)
                {
                    break;
                }

                chain.Add(angledDowns[angleIndex]);
                angledDowns.RemoveAt(angleIndex);
                currentVerticalIndex = nextVerticalIndex;
            }

            return chain;
        }

        private static int FindStartVertical(
            List<ClassifiedSegment> verticals,
            List<ClassifiedSegment> angledDowns)
        {
            int fallbackWithOutgoing = -1;
            int fallbackWithoutIncoming = -1;
            int fallbackAny = -1;

            for (int i = 0; i < verticals.Count; i++)
            {
                bool hasIncoming = HasIncomingAngledDown(verticals[i].Line.From, angledDowns);
                bool hasOutgoing = FindOutgoingAngleIndex(
                    verticals[i].Line.To,
                    verticals,
                    angledDowns,
                    out _) >= 0;

                if (!hasIncoming && hasOutgoing)
                {
                    return i;
                }

                if (fallbackWithOutgoing < 0 && hasOutgoing)
                {
                    fallbackWithOutgoing = i;
                }

                if (fallbackWithoutIncoming < 0 && !hasIncoming)
                {
                    fallbackWithoutIncoming = i;
                }

                if (fallbackAny < 0)
                {
                    fallbackAny = i;
                }
            }

            if (fallbackWithOutgoing >= 0)
            {
                return fallbackWithOutgoing;
            }

            if (fallbackWithoutIncoming >= 0)
            {
                return fallbackWithoutIncoming;
            }

            return fallbackAny;
        }

        private static int FindOutgoingAngleIndex(
            Point3d verticalEnd,
            List<ClassifiedSegment> verticals,
            List<ClassifiedSegment> angledDowns,
            out int nextVerticalIndex)
        {
            nextVerticalIndex = -1;
            for (int i = 0; i < angledDowns.Count; i++)
            {
                if (!PointsMatch(verticalEnd, angledDowns[i].Line.From))
                {
                    continue;
                }

                int candidateNextVerticalIndex = FindVerticalStartingAt(
                    angledDowns[i].Line.To,
                    verticals);

                if (candidateNextVerticalIndex >= 0)
                {
                    nextVerticalIndex = candidateNextVerticalIndex;
                    return i;
                }
            }

            return -1;
        }

        private static int FindVerticalStartingAt(
            Point3d point,
            List<ClassifiedSegment> verticals)
        {
            for (int i = 0; i < verticals.Count; i++)
            {
                if (PointsMatch(point, verticals[i].Line.From))
                {
                    return i;
                }
            }

            return -1;
        }

        private static bool HasIncomingAngledDown(
            Point3d verticalStart,
            List<ClassifiedSegment> angledDowns)
        {
            for (int i = 0; i < angledDowns.Count; i++)
            {
                if (PointsMatch(verticalStart, angledDowns[i].Line.To))
                {
                    return true;
                }
            }

            return false;
        }

        private static List<SMTClassifiedCurve> JoinAdjacentWovenMembers(List<SMTClassifiedCurve> wovenMembers, List<string> notices)
        {
            var joinedMembers = new List<SMTClassifiedCurve>();
            var currentPath = new List<ClassifiedSegment>();
            int joinedPairCount = 0;

            foreach (SMTClassifiedCurve member in wovenMembers)
            {
                List<ClassifiedSegment> nextPath = ExtractSegments(member).ToList();
                if (nextPath.Count == 0)
                {
                    continue;
                }

                if (currentPath.Count == 0)
                {
                    currentPath.AddRange(nextPath);
                    continue;
                }

                if (TryAppendConnectedPath(currentPath, nextPath))
                {
                    joinedPairCount++;
                    continue;
                }

                joinedMembers.Add(ToClassifiedCurve(currentPath));
                currentPath = nextPath;
            }

            if (currentPath.Count > 0)
            {
                joinedMembers.Add(ToClassifiedCurve(currentPath));
            }

            if (joinedPairCount > 0)
            {
                notices.Add($"Joined {joinedPairCount} connected adjacent member pair(s) after weaving.");
            }

            return joinedMembers;
        }

        private static List<SMTClassifiedCurve> ExplodeToClassifiedSegments(IEnumerable<SMTClassifiedCurve> continuousPaths)
        {
            var sortedSegments = new List<SMTClassifiedCurve>();
            foreach (SMTClassifiedCurve continuousPath in continuousPaths)
            {
                foreach (ClassifiedSegment segment in ExtractSegments(continuousPath))
                {
                    sortedSegments.Add(new SMTClassifiedCurve(new LineCurve(segment.Line), segment.Classification));
                }
            }

            return sortedSegments;
        }

        private static bool TryAppendConnectedPath(List<ClassifiedSegment> currentPath, List<ClassifiedSegment> nextPath)
        {
            ClassifiedSegment currentLast = currentPath[currentPath.Count - 1];
            ClassifiedSegment nextFirst = nextPath[0];
            ClassifiedSegment nextLast = nextPath[nextPath.Count - 1];

            if (PointsMatch(currentLast.Line.To, nextFirst.Line.From) &&
                CanJoinSegments(currentLast, nextFirst))
            {
                currentPath.AddRange(nextPath);
                return true;
            }

            if (PointsMatch(currentLast.Line.To, nextLast.Line.To) &&
                CanReversePathForJoin(nextPath) &&
                CanJoinSegments(currentLast, nextLast))
            {
                currentPath.AddRange(ReversePath(nextPath));
                return true;
            }

            return false;
        }

        private static bool CanJoinSegments(ClassifiedSegment first, ClassifiedSegment second)
        {
            if (first.Classification == SMTMemberClassification.Horizontal &&
                second.Classification == SMTMemberClassification.Horizontal)
            {
                return true;
            }

            return (first.Classification == SMTMemberClassification.Vertical && IsAngled(second.Classification)) ||
                   (IsAngled(first.Classification) && second.Classification == SMTMemberClassification.Vertical);
        }

        private static IEnumerable<ClassifiedSegment> ExtractSegments(SMTClassifiedCurve member)
        {
            if (member == null || member.Curve == null)
            {
                yield break;
            }

            if (member.Curve.TryGetPolyline(out Polyline polyline))
            {
                for (int i = 0; i < polyline.SegmentCount; i++)
                {
                    Line segment = polyline.SegmentAt(i);
                    if (segment.IsValid && segment.Length > RhinoMath.ZeroTolerance)
                    {
                        yield return new ClassifiedSegment
                        {
                            Line = segment,
                            Classification = member.GetClassificationForSegment(i)
                        };
                    }
                }

                yield break;
            }

            Line line = new Line(member.Curve.PointAtStart, member.Curve.PointAtEnd);
            if (line.IsValid && line.Length > RhinoMath.ZeroTolerance)
            {
                yield return new ClassifiedSegment
                {
                    Line = line,
                    Classification = member.Classification
                };
            }
        }

        private static SMTClassifiedCurve ToClassifiedCurve(List<ClassifiedSegment> segments)
        {
            if (segments.Count == 1)
            {
                ClassifiedSegment segment = segments[0];
                return new SMTClassifiedCurve(new LineCurve(segment.Line), segment.Classification);
            }

            var polyline = new Polyline();
            polyline.Add(segments[0].Line.From);
            foreach (ClassifiedSegment segment in segments)
            {
                polyline.Add(segment.Line.To);
            }

            List<SMTMemberClassification> classifications = segments
                .Select(segment => segment.Classification)
                .ToList();

            return new SMTClassifiedCurve(new PolylineCurve(polyline), classifications);
        }

        private static List<ClassifiedSegment> ReversePath(List<ClassifiedSegment> path)
        {
            var reversed = new List<ClassifiedSegment>();
            for (int i = path.Count - 1; i >= 0; i--)
            {
                reversed.Add(ReverseSegment(path[i]));
            }

            return reversed;
        }

        private static bool CanReversePathForJoin(List<ClassifiedSegment> path)
        {
            return !path.Any(segment => IsAngled(segment.Classification));
        }

        private static ClassifiedSegment ReverseSegment(ClassifiedSegment segment)
        {
            Line line = segment.Line;
            line.Flip();

            return new ClassifiedSegment
            {
                Line = line,
                Classification = segment.Classification
            };
        }

        private static bool PointsMatch(Point3d first, Point3d second)
        {
            return first.DistanceTo(second) <= ConnectionTolerance;
        }

        private static bool IsVerticalOrAngled(SMTMemberClassification classification)
        {
            return classification == SMTMemberClassification.Vertical ||
                   IsAngled(classification);
        }

        private static bool IsAngled(SMTMemberClassification classification)
        {
            return classification == SMTMemberClassification.Angled ||
                   classification == SMTMemberClassification.AngledDown;
        }

        private static bool SameLayerBand(Line first, Line second)
        {
            return Math.Abs(MinZ(first) - MinZ(second)) <= ConnectionTolerance &&
                   Math.Abs(MaxZ(first) - MaxZ(second)) <= ConnectionTolerance;
        }

        private static double MinZ(Line line)
        {
            return Math.Min(line.From.Z, line.To.Z);
        }

        private static double MaxZ(Line line)
        {
            return Math.Max(line.From.Z, line.To.Z);
        }

        private static string BuildReport(
            List<SMTMemberClassification> pattern,
            int horizontalCount,
            int verticalCount,
            int angledCount,
            int outputCount,
            List<string> notices)
        {
            var lines = new List<string>
            {
                "Pattern: connected horizontal group -> attached vertical/angled paths, repeated by connectivity",
                "Horizontal members: " + horizontalCount,
                "Vertical members: " + verticalCount,
                "Angled members: " + angledCount,
                "Sorted output members: " + outputCount
            };

            lines.AddRange(notices);
            return string.Join(Environment.NewLine, lines);
        }

        protected override System.Drawing.Bitmap Icon => null;

        public override Guid ComponentGuid => new Guid("bc78ef05-812a-49c5-a204-a8156b77b51b");
    }
}
