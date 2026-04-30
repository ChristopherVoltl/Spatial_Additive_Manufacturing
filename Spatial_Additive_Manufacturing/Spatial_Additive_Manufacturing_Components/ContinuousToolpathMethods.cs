using System;
using System.Collections.Generic;
using System.Linq;
using Grasshopper;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Rhino;
using Rhino.Geometry;

namespace Spatial_Additive_Manufacturing.Spatial_Printing_Components
{
    public class ContinuousToolpathMethods : GH_Component
    {
        public ContinuousToolpathMethods()
          : base("Continuous Toolpath Methods", "CTM",
              "Builds a graph from warped-grid members, classifies horizontal and vertical elements, and generates diagonal bracing between neighboring vertical struts.",
              "FGAM", "Toolpathing")
        {
        }

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddCurveParameter("Grid Curves", "G", "Warped-grid curves. These are treated as horizontal members when inter-layer struts are supplied.", GH_ParamAccess.list);
            pManager.AddNumberParameter("Tolerance", "T", "Node merge tolerance used for graph reconstruction.", GH_ParamAccess.item, 0.01);
            pManager.AddLineParameter("Inter-Layer Struts", "S", "Optional warped-grid struts. When connected, these are treated as the vertical members of the graph.", GH_ParamAccess.list);
            pManager[2].Optional = true;
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddLineParameter("Graph Members", "GM", "All horizontal, vertical, and generated angled members in the graph.", GH_ParamAccess.tree);
            pManager.AddPointParameter("Graph Nodes", "GN", "Merged graph nodes.", GH_ParamAccess.tree);
            pManager.AddLineParameter("Horizontal Elements", "HE", "Exploded horizontal members derived from the warped-grid curves.", GH_ParamAccess.tree);
            pManager.AddLineParameter("Vertical Elements", "VE", "Vertical members derived from inter-layer struts or inferred from the input.", GH_ParamAccess.tree);
            pManager.AddLineParameter("Angled Elements", "AE", "Generated diagonal members that brace neighboring vertical struts.", GH_ParamAccess.tree);
            pManager.AddTextParameter("Analysis", "A", "Diagnostic summary of the reconstructed graph.", GH_ParamAccess.item);
        }

        private enum MemberType
        {
            Horizontal,
            Vertical,
            Angled
        }

        private sealed class GraphNode
        {
            public int Id;
            public Point3d Point;
        }

        private sealed class GraphEdge
        {
            public int Id;
            public int A;
            public int B;
            public int LowNodeId;
            public int HighNodeId;
            public Line Line;
            public MemberType Type;
        }

        private sealed class CandidateFace
        {
            public int Id;
            public GraphEdge LowerHorizontal;
            public GraphEdge UpperHorizontal;
            public GraphEdge VerticalA;
            public GraphEdge VerticalB;
            public Vector3d Direction;
        }

        private sealed class OrientedFace
        {
            public CandidateFace Face;
            public int LeftVerticalId;
            public int RightVerticalId;
            public int LowerStartNodeId;
            public int LowerEndNodeId;
            public int UpperStartNodeId;
            public int UpperEndNodeId;
        }

        private sealed class FaceNeighbor
        {
            public int NeighborFaceId;
            public int SharedVerticalId;
        }

        private sealed class BraceBand
        {
            public int Id;
            public Vector3d Direction;
            public List<OrientedFace> Faces = new List<OrientedFace>();
        }

        private sealed class CandidateDiagonal
        {
            public int Id;
            public int FaceId;
            public int BandId;
            public int LayoutId;
            public int BottomVerticalId;
            public int TopVerticalId;
            public int BottomNodeId;
            public int TopNodeId;
            public Line Line;
            public HashSet<int> Conflicts = new HashSet<int>();
        }

        private sealed class SelectionMetrics
        {
            public int LongestDiagonalPathCount;
            public int LongestPathMemberCount;
            public int PathComponentCount;
            public int FullyBracedVerticalCount;
            public int TouchedVerticalCount;
            public int DiagonalCount;
            public int TotalPathDiagonalCount;
            public int TotalPathMemberCount;
            public List<int> OrderedPathDiagonalCounts = new List<int>();
            public List<int> OrderedPathMemberCounts = new List<int>();
            public int DominantOrientationBandCount;
            public int OpposingOrientationBandCount;
        }

        private sealed class DiagonalSelectionResult
        {
            public List<CandidateDiagonal> Selected = new List<CandidateDiagonal>();
            public List<BandLayout> SelectedLayouts = new List<BandLayout>();
            public SelectionMetrics Metrics = new SelectionMetrics();
            public int SelectedBandCount;
        }

        private sealed class BandLayout
        {
            public int LayoutId;
            public int BandId;
            public bool UseForwardPattern;
            public bool NormalizedForwardPattern;
            public List<CandidateDiagonal> Diagonals = new List<CandidateDiagonal>();
        }

        private sealed class BracedGraph
        {
            private readonly double _tolerance;
            private readonly double _toleranceSquared;
            private readonly Dictionary<Tuple<int, int, int>, List<int>> _nodeBuckets;
            private readonly HashSet<string> _memberKeys;

            public BracedGraph(double tolerance)
            {
                _tolerance = Math.Max(tolerance, 1e-6);
                _toleranceSquared = _tolerance * _tolerance;
                _nodeBuckets = new Dictionary<Tuple<int, int, int>, List<int>>();
                _memberKeys = new HashSet<string>(StringComparer.Ordinal);
                Nodes = new List<GraphNode>();
                HorizontalEdges = new List<GraphEdge>();
                VerticalEdges = new List<GraphEdge>();
                AngledEdges = new List<GraphEdge>();
            }

            public List<GraphNode> Nodes { get; }
            public List<GraphEdge> HorizontalEdges { get; }
            public List<GraphEdge> VerticalEdges { get; }
            public List<GraphEdge> AngledEdges { get; }
            public int BracedFaceCount { get; private set; }
            public int TouchedVerticalCount { get; private set; }
            public int FullyBracedVerticalCount { get; private set; }
            public int LongestDiagonalPathCount { get; private set; }
            public int LongestPathMemberCount { get; private set; }

            public IEnumerable<GraphEdge> AllEdges
            {
                get
                {
                    foreach (GraphEdge edge in HorizontalEdges) yield return edge;
                    foreach (GraphEdge edge in VerticalEdges) yield return edge;
                    foreach (GraphEdge edge in AngledEdges) yield return edge;
                }
            }

            public void AddHorizontalCurve(Curve curve)
            {
                foreach (Line segment in ExtractSegments(curve))
                {
                    AddMember(segment, MemberType.Horizontal);
                }
            }

            public void AddFallbackCurve(Curve curve)
            {
                foreach (Line segment in ExtractSegments(curve))
                {
                    MemberType type = ClassifySegment(segment);
                    AddMember(segment, type);
                }
            }

            public void AddVerticalLine(Line line)
            {
                AddMember(line, MemberType.Vertical);
            }

            public void BuildSingleBraceBands()
            {
                List<CandidateFace> candidateFaces = BuildCandidateFaces();
                if (candidateFaces.Count == 0)
                {
                    return;
                }

                BracedFaceCount = 0;
                TouchedVerticalCount = 0;
                FullyBracedVerticalCount = 0;
                LongestDiagonalPathCount = 0;
                LongestPathMemberCount = 0;

                foreach (List<CandidateFace> layerComponent in BuildLayerComponents(candidateFaces))
                {
                    List<BraceBand> braceBands = BuildBraceBands(layerComponent);
                    if (braceBands.Count == 0)
                    {
                        continue;
                    }

                    DiagonalSelectionResult optimizedSelection = SelectBestComponentLayout(braceBands);
                    TouchedVerticalCount += optimizedSelection.Metrics.TouchedVerticalCount;
                    FullyBracedVerticalCount += optimizedSelection.Metrics.FullyBracedVerticalCount;
                    LongestDiagonalPathCount = Math.Max(LongestDiagonalPathCount, optimizedSelection.Metrics.LongestDiagonalPathCount);
                    LongestPathMemberCount = Math.Max(LongestPathMemberCount, optimizedSelection.Metrics.LongestPathMemberCount);

                    foreach (CandidateDiagonal candidate in optimizedSelection.Selected)
                    {
                        if (AddMember(candidate.Line, MemberType.Angled) != null)
                        {
                            BracedFaceCount++;
                        }
                    }
                }
            }

            private List<CandidateFace> BuildCandidateFaces()
            {
                var faces = new List<CandidateFace>();
                var horizontalLookup = new Dictionary<string, GraphEdge>(StringComparer.Ordinal);
                foreach (GraphEdge horizontal in HorizontalEdges)
                {
                    string key = BuildNodePairKey(horizontal.A, horizontal.B);
                    if (!horizontalLookup.ContainsKey(key))
                    {
                        horizontalLookup[key] = horizontal;
                    }
                }

                var verticalsByLowNode = new Dictionary<int, List<GraphEdge>>();
                foreach (GraphEdge vertical in VerticalEdges)
                {
                    if (!verticalsByLowNode.TryGetValue(vertical.LowNodeId, out List<GraphEdge> bucket))
                    {
                        bucket = new List<GraphEdge>();
                        verticalsByLowNode[vertical.LowNodeId] = bucket;
                    }

                    bucket.Add(vertical);
                }

                var processedFaces = new HashSet<string>(StringComparer.Ordinal);
                int nextFaceId = 0;

                foreach (GraphEdge lowerHorizontal in HorizontalEdges)
                {
                    if (!verticalsByLowNode.TryGetValue(lowerHorizontal.A, out List<GraphEdge> startVerticals) ||
                        !verticalsByLowNode.TryGetValue(lowerHorizontal.B, out List<GraphEdge> endVerticals))
                    {
                        continue;
                    }

                    foreach (GraphEdge verticalA in startVerticals)
                    {
                        foreach (GraphEdge verticalB in endVerticals)
                        {
                            if (verticalA.Id == verticalB.Id)
                            {
                                continue;
                            }

                            string upperPairKey = BuildNodePairKey(verticalA.HighNodeId, verticalB.HighNodeId);
                            if (!horizontalLookup.TryGetValue(upperPairKey, out GraphEdge upperHorizontal))
                            {
                                continue;
                            }

                            string faceKey = BuildFaceKey(
                                verticalA.LowNodeId,
                                verticalB.LowNodeId,
                                verticalA.HighNodeId,
                                verticalB.HighNodeId);

                            if (!processedFaces.Add(faceKey))
                            {
                                continue;
                            }

                            faces.Add(new CandidateFace
                            {
                                Id = nextFaceId++,
                                LowerHorizontal = lowerHorizontal,
                                UpperHorizontal = upperHorizontal,
                                VerticalA = verticalA,
                                VerticalB = verticalB,
                                Direction = AverageDirection(lowerHorizontal.Line.Direction, upperHorizontal.Line.Direction)
                            });
                        }
                    }
                }

                return faces;
            }

            private List<List<CandidateFace>> BuildLayerComponents(List<CandidateFace> candidateFaces)
            {
                var facesByVertical = new Dictionary<int, List<int>>();
                var faceLookup = candidateFaces.ToDictionary(face => face.Id);
                var faceAdjacency = candidateFaces.ToDictionary(face => face.Id, face => new HashSet<int>());

                foreach (CandidateFace face in candidateFaces)
                {
                    AddFaceToVerticalBucket(face.VerticalA.Id, face.Id, facesByVertical);
                    AddFaceToVerticalBucket(face.VerticalB.Id, face.Id, facesByVertical);
                }

                foreach (KeyValuePair<int, List<int>> bucket in facesByVertical)
                {
                    List<int> faceIds = bucket.Value;
                    for (int indexA = 0; indexA < faceIds.Count; indexA++)
                    {
                        for (int indexB = indexA + 1; indexB < faceIds.Count; indexB++)
                        {
                            faceAdjacency[faceIds[indexA]].Add(faceIds[indexB]);
                            faceAdjacency[faceIds[indexB]].Add(faceIds[indexA]);
                        }
                    }
                }

                var unvisitedFaceIds = new HashSet<int>(candidateFaces.Select(face => face.Id));
                var components = new List<List<CandidateFace>>();

                while (unvisitedFaceIds.Count > 0)
                {
                    int seedFaceId = unvisitedFaceIds.First();
                    var stack = new Stack<int>();
                    var component = new List<CandidateFace>();
                    stack.Push(seedFaceId);

                    while (stack.Count > 0)
                    {
                        int currentFaceId = stack.Pop();
                        if (!unvisitedFaceIds.Remove(currentFaceId))
                        {
                            continue;
                        }

                        component.Add(faceLookup[currentFaceId]);
                        foreach (int neighborFaceId in faceAdjacency[currentFaceId])
                        {
                            if (unvisitedFaceIds.Contains(neighborFaceId))
                            {
                                stack.Push(neighborFaceId);
                            }
                        }
                    }

                    if (component.Count > 0)
                    {
                        components.Add(component);
                    }
                }

                return components;
            }

            private DiagonalSelectionResult SelectBestComponentLayout(List<BraceBand> braceBands)
            {
                Vector3d layerReferenceDirection = AverageDirection(braceBands.Select(band => band.Direction));
                List<List<BandLayout>> layoutOptionsByBand = BuildLayoutOptionsByBand(braceBands, layerReferenceDirection);
                DiagonalSelectionResult bestSelection = SearchBestSelection(
                    layoutOptionsByBand,
                    new List<CandidateDiagonal>(),
                    new HashSet<int>(),
                    new HashSet<int>(),
                    CreateSelectionResult,
                    IsBetterPrimarySelection);

                AugmentSelectionWithAdditionalChains(layoutOptionsByBand, braceBands, bestSelection);
                bestSelection.Metrics = EvaluateSelection(bestSelection.Selected, bestSelection.SelectedLayouts);
                bestSelection.SelectedBandCount = bestSelection.SelectedLayouts.Count;
                return bestSelection;
            }

            private List<List<BandLayout>> BuildLayoutOptionsByBand(List<BraceBand> braceBands, Vector3d layerReferenceDirection)
            {
                List<BraceBand> orderedBands = braceBands
                    .OrderByDescending(band => band.Faces.Count)
                    .ThenBy(band => band.Id)
                    .ToList();

                var layoutOptionsByBand = new List<List<BandLayout>>();
                int nextLayoutId = 0;
                foreach (BraceBand band in orderedBands)
                {
                    layoutOptionsByBand.Add(new List<BandLayout>
                    {
                        CreateBandLayout(band, true, layerReferenceDirection, nextLayoutId++),
                        CreateBandLayout(band, false, layerReferenceDirection, nextLayoutId++)
                    });
                }

                return layoutOptionsByBand;
            }

            private int[] BuildRemainingDiagonalCounts(List<List<BandLayout>> layoutOptionsByBand)
            {
                var remainingCounts = new int[layoutOptionsByBand.Count + 1];

                for (int bandIndex = layoutOptionsByBand.Count - 1; bandIndex >= 0; bandIndex--)
                {
                    int bandDiagonalCount = layoutOptionsByBand[bandIndex].Count > 0
                        ? layoutOptionsByBand[bandIndex][0].Diagonals.Count
                        : 0;
                    remainingCounts[bandIndex] = remainingCounts[bandIndex + 1] + bandDiagonalCount;
                }

                return remainingCounts;
            }

            private BandLayout CreateBandLayout(BraceBand band, bool useForwardPattern, Vector3d layerReferenceDirection, int layoutId)
            {
                var layout = new BandLayout
                {
                    LayoutId = layoutId,
                    BandId = band.Id,
                    UseForwardPattern = useForwardPattern,
                    NormalizedForwardPattern = NormalizeBandPatternDirection(layerReferenceDirection, band.Direction, useForwardPattern)
                };

                foreach (OrientedFace face in band.Faces)
                {
                    int bottomVerticalId = useForwardPattern ? face.LeftVerticalId : face.RightVerticalId;
                    int topVerticalId = useForwardPattern ? face.RightVerticalId : face.LeftVerticalId;
                    int bottomNodeId = useForwardPattern ? face.LowerStartNodeId : face.LowerEndNodeId;
                    int topNodeId = useForwardPattern ? face.UpperEndNodeId : face.UpperStartNodeId;

                    layout.Diagonals.Add(new CandidateDiagonal
                    {
                        Id = layout.Diagonals.Count,
                        FaceId = face.Face.Id,
                        BandId = band.Id,
                        LayoutId = layoutId,
                        BottomVerticalId = bottomVerticalId,
                        TopVerticalId = topVerticalId,
                        BottomNodeId = bottomNodeId,
                        TopNodeId = topNodeId,
                        Line = new Line(Nodes[bottomNodeId].Point, Nodes[topNodeId].Point)
                    });
                }

                return layout;
            }

            private DiagonalSelectionResult SearchBestSelection(
                List<List<BandLayout>> layoutOptionsByBand,
                List<CandidateDiagonal> fixedSelection,
                HashSet<int> usedBottomVerticals,
                HashSet<int> usedTopVerticals,
                Func<List<CandidateDiagonal>, List<BandLayout>, DiagonalSelectionResult> evaluateResult,
                Func<DiagonalSelectionResult, DiagonalSelectionResult, bool> isBetterSelection)
            {
                int[] remainingDiagonalCounts = BuildRemainingDiagonalCounts(layoutOptionsByBand);
                var bestSelection = new DiagonalSelectionResult();
                SearchComponentLayouts(
                    0,
                    layoutOptionsByBand,
                    remainingDiagonalCounts,
                    new List<CandidateDiagonal>(),
                    new List<BandLayout>(),
                    fixedSelection,
                    new HashSet<int>(usedBottomVerticals),
                    new HashSet<int>(usedTopVerticals),
                    bestSelection,
                    evaluateResult,
                    isBetterSelection);

                return bestSelection;
            }

            private void AugmentSelectionWithAdditionalChains(
                List<List<BandLayout>> layoutOptionsByBand,
                List<BraceBand> braceBands,
                DiagonalSelectionResult accumulatedSelection)
            {
                var componentVerticalIds = new HashSet<int>(
                    braceBands.SelectMany(band => band.Faces.SelectMany(face => new[] { face.LeftVerticalId, face.RightVerticalId })));

                var usedBottomVerticals = new HashSet<int>(accumulatedSelection.Selected.Select(diagonal => diagonal.BottomVerticalId));
                var usedTopVerticals = new HashSet<int>(accumulatedSelection.Selected.Select(diagonal => diagonal.TopVerticalId));
                HashSet<int> touchedVerticals = BuildTouchedVerticalSet(accumulatedSelection.Selected);

                while (true)
                {
                    var untouchedVerticals = new HashSet<int>(componentVerticalIds.Where(verticalId => !touchedVerticals.Contains(verticalId)));
                    if (untouchedVerticals.Count == 0)
                    {
                        break;
                    }

                    List<List<BandLayout>> supplementalLayoutOptions = BuildSupplementalLayoutOptions(
                        layoutOptionsByBand,
                        accumulatedSelection.Selected,
                        usedBottomVerticals,
                        usedTopVerticals);

                    if (supplementalLayoutOptions.Count == 0)
                    {
                        break;
                    }

                    DiagonalSelectionResult supplementalSelection = SearchBestSelection(
                        supplementalLayoutOptions,
                        accumulatedSelection.Selected,
                        usedBottomVerticals,
                        usedTopVerticals,
                        (diagonals, layouts) => ExtractBestUntouchedChain(diagonals, layouts, untouchedVerticals),
                        (candidate, best) => IsBetterSupplementalSelection(candidate, best, untouchedVerticals));

                    if (supplementalSelection.Selected.Count == 0)
                    {
                        break;
                    }

                    accumulatedSelection.Selected.AddRange(supplementalSelection.Selected);
                    accumulatedSelection.SelectedLayouts.AddRange(supplementalSelection.SelectedLayouts);
                    accumulatedSelection.SelectedBandCount = accumulatedSelection.SelectedLayouts.Count;

                    foreach (CandidateDiagonal diagonal in supplementalSelection.Selected)
                    {
                        usedBottomVerticals.Add(diagonal.BottomVerticalId);
                        usedTopVerticals.Add(diagonal.TopVerticalId);
                        touchedVerticals.Add(diagonal.BottomVerticalId);
                        touchedVerticals.Add(diagonal.TopVerticalId);
                    }
                }
            }

            private List<List<BandLayout>> BuildSupplementalLayoutOptions(
                List<List<BandLayout>> baseLayoutOptions,
                List<CandidateDiagonal> fixedSelection,
                HashSet<int> usedBottomVerticals,
                HashSet<int> usedTopVerticals)
            {
                var supplementalLayoutOptions = new List<List<BandLayout>>();
                int nextLayoutId = GetNextLayoutId(baseLayoutOptions, fixedSelection);

                foreach (List<BandLayout> layoutOptions in baseLayoutOptions)
                {
                    foreach (BandLayout layout in layoutOptions)
                    {
                        foreach (BandLayout segmentLayout in ExtractValidSegments(
                            layout,
                            fixedSelection,
                            usedBottomVerticals,
                            usedTopVerticals,
                            ref nextLayoutId))
                        {
                            supplementalLayoutOptions.Add(new List<BandLayout> { segmentLayout });
                        }
                    }
                }

                return supplementalLayoutOptions;
            }

            private int GetNextLayoutId(IEnumerable<List<BandLayout>> layoutOptionsByBand, IEnumerable<CandidateDiagonal> fixedSelection)
            {
                int maxLayoutId = -1;

                foreach (List<BandLayout> layoutOptions in layoutOptionsByBand)
                {
                    foreach (BandLayout layout in layoutOptions)
                    {
                        maxLayoutId = Math.Max(maxLayoutId, layout.LayoutId);
                    }
                }

                foreach (CandidateDiagonal diagonal in fixedSelection)
                {
                    maxLayoutId = Math.Max(maxLayoutId, diagonal.LayoutId);
                }

                return maxLayoutId + 1;
            }

            private List<BandLayout> ExtractValidSegments(
                BandLayout sourceLayout,
                List<CandidateDiagonal> fixedSelection,
                HashSet<int> usedBottomVerticals,
                HashSet<int> usedTopVerticals,
                ref int nextLayoutId)
            {
                var segmentLayouts = new List<BandLayout>();
                BandLayout currentSegment = null;

                foreach (CandidateDiagonal diagonal in sourceLayout.Diagonals)
                {
                    bool isValid =
                        !usedBottomVerticals.Contains(diagonal.BottomVerticalId) &&
                        !usedTopVerticals.Contains(diagonal.TopVerticalId) &&
                        fixedSelection.All(selected => !DiagonalsIntersectInInterior(diagonal, selected));

                    if (!isValid)
                    {
                        FinalizeSegment(currentSegment, segmentLayouts);
                        currentSegment = null;
                        continue;
                    }

                    if (currentSegment == null ||
                        (currentSegment.Diagonals.Count > 0 &&
                         !ConnectsThroughVertical(currentSegment.Diagonals[currentSegment.Diagonals.Count - 1], diagonal)))
                    {
                        FinalizeSegment(currentSegment, segmentLayouts);
                        currentSegment = new BandLayout
                        {
                            LayoutId = nextLayoutId++,
                            BandId = sourceLayout.BandId,
                            UseForwardPattern = sourceLayout.UseForwardPattern,
                            NormalizedForwardPattern = sourceLayout.NormalizedForwardPattern
                        };
                    }

                    currentSegment.Diagonals.Add(CloneDiagonalForLayout(diagonal, currentSegment.LayoutId, currentSegment.Diagonals.Count));
                }

                FinalizeSegment(currentSegment, segmentLayouts);
                return segmentLayouts;
            }

            private void FinalizeSegment(BandLayout segment, List<BandLayout> segments)
            {
                if (segment != null && segment.Diagonals.Count > 0)
                {
                    segments.Add(segment);
                }
            }

            private CandidateDiagonal CloneDiagonalForLayout(CandidateDiagonal source, int layoutId, int localId)
            {
                return new CandidateDiagonal
                {
                    Id = localId,
                    FaceId = source.FaceId,
                    BandId = source.BandId,
                    LayoutId = layoutId,
                    BottomVerticalId = source.BottomVerticalId,
                    TopVerticalId = source.TopVerticalId,
                    BottomNodeId = source.BottomNodeId,
                    TopNodeId = source.TopNodeId,
                    Line = source.Line
                };
            }

            private bool NormalizeBandPatternDirection(Vector3d layerReferenceDirection, Vector3d bandDirection, bool useForwardPattern)
            {
                Vector3d reference = layerReferenceDirection;
                Vector3d band = bandDirection;

                if (!reference.Unitize() || !band.Unitize())
                {
                    return useForwardPattern;
                }

                return (reference * band) >= 0.0 ? useForwardPattern : !useForwardPattern;
            }

            private void SearchComponentLayouts(
                int bandIndex,
                List<List<BandLayout>> layoutOptionsByBand,
                int[] remainingDiagonalCounts,
                List<CandidateDiagonal> currentSelection,
                List<BandLayout> selectedLayouts,
                List<CandidateDiagonal> fixedSelection,
                HashSet<int> usedBottomVerticals,
                HashSet<int> usedTopVerticals,
                DiagonalSelectionResult bestSelection,
                Func<List<CandidateDiagonal>, List<BandLayout>, DiagonalSelectionResult> evaluateResult,
                Func<DiagonalSelectionResult, DiagonalSelectionResult, bool> isBetterSelection)
            {
                if (bandIndex < layoutOptionsByBand.Count &&
                    (currentSelection.Count + remainingDiagonalCounts[bandIndex]) < bestSelection.Metrics.LongestDiagonalPathCount)
                {
                    return;
                }

                if (bandIndex >= layoutOptionsByBand.Count)
                {
                    DiagonalSelectionResult currentResult = evaluateResult(currentSelection, selectedLayouts);

                    if (isBetterSelection(currentResult, bestSelection))
                    {
                        bestSelection.Selected = currentResult.Selected;
                        bestSelection.SelectedLayouts = currentResult.SelectedLayouts;
                        bestSelection.Metrics = currentResult.Metrics;
                        bestSelection.SelectedBandCount = currentResult.SelectedBandCount;
                    }

                    return;
                }

                foreach (BandLayout layout in OrderBandLayoutsForSearch(layoutOptionsByBand[bandIndex], currentSelection))
                {
                    if (!CanApplyLayout(layout, fixedSelection, currentSelection, usedBottomVerticals, usedTopVerticals))
                    {
                        continue;
                    }

                    int originalSelectionCount = currentSelection.Count;
                    var addedBottomVerticals = new List<int>();
                    var addedTopVerticals = new List<int>();

                    foreach (CandidateDiagonal diagonal in layout.Diagonals)
                    {
                        currentSelection.Add(diagonal);

                        if (usedBottomVerticals.Add(diagonal.BottomVerticalId))
                        {
                            addedBottomVerticals.Add(diagonal.BottomVerticalId);
                        }

                        if (usedTopVerticals.Add(diagonal.TopVerticalId))
                        {
                            addedTopVerticals.Add(diagonal.TopVerticalId);
                        }
                    }

                    selectedLayouts.Add(layout);
                    SearchComponentLayouts(
                        bandIndex + 1,
                        layoutOptionsByBand,
                        remainingDiagonalCounts,
                        currentSelection,
                        selectedLayouts,
                        fixedSelection,
                        usedBottomVerticals,
                        usedTopVerticals,
                        bestSelection,
                        evaluateResult,
                        isBetterSelection);
                    selectedLayouts.RemoveAt(selectedLayouts.Count - 1);

                    currentSelection.RemoveRange(originalSelectionCount, currentSelection.Count - originalSelectionCount);
                    foreach (int verticalId in addedBottomVerticals)
                    {
                        usedBottomVerticals.Remove(verticalId);
                    }

                    foreach (int verticalId in addedTopVerticals)
                    {
                        usedTopVerticals.Remove(verticalId);
                    }
                }

                SearchComponentLayouts(
                    bandIndex + 1,
                    layoutOptionsByBand,
                    remainingDiagonalCounts,
                    currentSelection,
                    selectedLayouts,
                    fixedSelection,
                    usedBottomVerticals,
                    usedTopVerticals,
                    bestSelection,
                    evaluateResult,
                    isBetterSelection);
            }

            private List<BandLayout> OrderBandLayoutsForSearch(List<BandLayout> layouts, List<CandidateDiagonal> currentSelection)
            {
                return layouts
                    .OrderByDescending(layout => CountLayoutConnections(layout, currentSelection))
                    .ThenByDescending(layout => layout.Diagonals.Count)
                    .ThenByDescending(layout => layout.NormalizedForwardPattern ? 1 : 0)
                    .ToList();
            }

            private int CountLayoutConnections(BandLayout layout, List<CandidateDiagonal> currentSelection)
            {
                int connectionCount = 0;

                foreach (CandidateDiagonal candidate in layout.Diagonals)
                {
                    if (currentSelection.Any(selected => ConnectsThroughVertical(candidate, selected)))
                    {
                        connectionCount++;
                    }
                }

                return connectionCount;
            }

            private bool IsBetterPrimarySelection(DiagonalSelectionResult candidateResult, DiagonalSelectionResult bestResult)
            {
                SelectionMetrics candidate = candidateResult.Metrics;
                SelectionMetrics best = bestResult.Metrics;

                if (candidate.LongestDiagonalPathCount != best.LongestDiagonalPathCount)
                {
                    return candidate.LongestDiagonalPathCount > best.LongestDiagonalPathCount;
                }

                if (candidate.LongestPathMemberCount != best.LongestPathMemberCount)
                {
                    return candidate.LongestPathMemberCount > best.LongestPathMemberCount;
                }

                if (candidate.PathComponentCount != best.PathComponentCount)
                {
                    return candidate.PathComponentCount < best.PathComponentCount;
                }

                if (candidate.FullyBracedVerticalCount != best.FullyBracedVerticalCount)
                {
                    return candidate.FullyBracedVerticalCount > best.FullyBracedVerticalCount;
                }

                if (candidate.TouchedVerticalCount != best.TouchedVerticalCount)
                {
                    return candidate.TouchedVerticalCount > best.TouchedVerticalCount;
                }

                if (candidate.DiagonalCount != best.DiagonalCount)
                {
                    return candidate.DiagonalCount > best.DiagonalCount;
                }

                if (candidate.DominantOrientationBandCount != best.DominantOrientationBandCount)
                {
                    return candidate.DominantOrientationBandCount > best.DominantOrientationBandCount;
                }

                if (candidate.OpposingOrientationBandCount != best.OpposingOrientationBandCount)
                {
                    return candidate.OpposingOrientationBandCount < best.OpposingOrientationBandCount;
                }

                if (candidateResult.SelectedBandCount != bestResult.SelectedBandCount)
                {
                    return candidateResult.SelectedBandCount < bestResult.SelectedBandCount;
                }

                return false;
            }

            private bool IsBetterSupplementalSelection(
                DiagonalSelectionResult candidateResult,
                DiagonalSelectionResult bestResult,
                HashSet<int> untouchedVerticals)
            {
                int candidateUntouchedCount = CountTouchedVerticals(candidateResult.Selected, untouchedVerticals);
                int bestUntouchedCount = CountTouchedVerticals(bestResult.Selected, untouchedVerticals);
                bool candidateIsValid = candidateUntouchedCount > 0;
                bool bestIsValid = bestUntouchedCount > 0;

                if (candidateIsValid != bestIsValid)
                {
                    return candidateIsValid;
                }

                if (!candidateIsValid)
                {
                    return false;
                }

                SelectionMetrics candidate = candidateResult.Metrics;
                SelectionMetrics best = bestResult.Metrics;

                if (candidate.LongestDiagonalPathCount != best.LongestDiagonalPathCount)
                {
                    return candidate.LongestDiagonalPathCount > best.LongestDiagonalPathCount;
                }

                if (candidate.LongestPathMemberCount != best.LongestPathMemberCount)
                {
                    return candidate.LongestPathMemberCount > best.LongestPathMemberCount;
                }

                if (candidateUntouchedCount != bestUntouchedCount)
                {
                    return candidateUntouchedCount > bestUntouchedCount;
                }

                if (candidate.DiagonalCount != best.DiagonalCount)
                {
                    return candidate.DiagonalCount > best.DiagonalCount;
                }

                if (candidate.DominantOrientationBandCount != best.DominantOrientationBandCount)
                {
                    return candidate.DominantOrientationBandCount > best.DominantOrientationBandCount;
                }

                if (candidate.OpposingOrientationBandCount != best.OpposingOrientationBandCount)
                {
                    return candidate.OpposingOrientationBandCount < best.OpposingOrientationBandCount;
                }

                return candidateResult.SelectedBandCount < bestResult.SelectedBandCount;
            }

            private DiagonalSelectionResult CreateSelectionResult(List<CandidateDiagonal> diagonals, List<BandLayout> selectedLayouts)
            {
                return new DiagonalSelectionResult
                {
                    Selected = new List<CandidateDiagonal>(diagonals),
                    SelectedLayouts = new List<BandLayout>(selectedLayouts),
                    Metrics = EvaluateSelection(diagonals, selectedLayouts),
                    SelectedBandCount = selectedLayouts.Count
                };
            }

            private bool CanApplyLayout(
                BandLayout layout,
                List<CandidateDiagonal> fixedSelection,
                List<CandidateDiagonal> currentSelection,
                HashSet<int> usedBottomVerticals,
                HashSet<int> usedTopVerticals)
            {
                foreach (CandidateDiagonal candidate in layout.Diagonals)
                {
                    if (usedBottomVerticals.Contains(candidate.BottomVerticalId) ||
                        usedTopVerticals.Contains(candidate.TopVerticalId))
                    {
                        return false;
                    }

                    foreach (CandidateDiagonal selected in currentSelection)
                    {
                        if (DiagonalsIntersectInInterior(candidate, selected))
                        {
                            return false;
                        }
                    }

                    foreach (CandidateDiagonal fixedDiagonal in fixedSelection)
                    {
                        if (DiagonalsIntersectInInterior(candidate, fixedDiagonal))
                        {
                            return false;
                        }
                    }
                }

                return true;
            }

            private List<CandidateDiagonal> BuildCandidateDiagonals(List<CandidateFace> candidateFaces)
            {
                var diagonals = new List<CandidateDiagonal>();

                foreach (CandidateFace face in candidateFaces)
                {
                    diagonals.Add(new CandidateDiagonal
                    {
                        Id = diagonals.Count,
                        FaceId = face.Id,
                        BottomVerticalId = face.VerticalA.Id,
                        TopVerticalId = face.VerticalB.Id,
                        BottomNodeId = face.VerticalA.LowNodeId,
                        TopNodeId = face.VerticalB.HighNodeId,
                        Line = new Line(Nodes[face.VerticalA.LowNodeId].Point, Nodes[face.VerticalB.HighNodeId].Point)
                    });

                    diagonals.Add(new CandidateDiagonal
                    {
                        Id = diagonals.Count,
                        FaceId = face.Id,
                        BottomVerticalId = face.VerticalB.Id,
                        TopVerticalId = face.VerticalA.Id,
                        BottomNodeId = face.VerticalB.LowNodeId,
                        TopNodeId = face.VerticalA.HighNodeId,
                        Line = new Line(Nodes[face.VerticalB.LowNodeId].Point, Nodes[face.VerticalA.HighNodeId].Point)
                    });
                }

                return diagonals;
            }

            private DiagonalSelectionResult SelectOptimizedDiagonals(List<CandidateDiagonal> candidateDiagonals)
            {
                BuildDiagonalConflicts(candidateDiagonals);

                var results = new List<DiagonalSelectionResult>
                {
                    RunGreedySelection(candidateDiagonals, 0),
                    RunGreedySelection(candidateDiagonals, 1),
                    RunGreedySelection(candidateDiagonals, 2)
                };

                return results
                    .OrderByDescending(result => result.Metrics.FullyBracedVerticalCount)
                    .ThenByDescending(result => result.Metrics.TouchedVerticalCount)
                    .ThenByDescending(result => result.Metrics.DiagonalCount)
                    .First();
            }

            private void BuildDiagonalConflicts(List<CandidateDiagonal> candidateDiagonals)
            {
                for (int indexA = 0; indexA < candidateDiagonals.Count; indexA++)
                {
                    CandidateDiagonal diagonalA = candidateDiagonals[indexA];

                    for (int indexB = indexA + 1; indexB < candidateDiagonals.Count; indexB++)
                    {
                        CandidateDiagonal diagonalB = candidateDiagonals[indexB];

                        if (diagonalA.FaceId == diagonalB.FaceId ||
                            diagonalA.BottomVerticalId == diagonalB.BottomVerticalId ||
                            diagonalA.TopVerticalId == diagonalB.TopVerticalId ||
                            DiagonalsIntersectInInterior(diagonalA, diagonalB))
                        {
                            diagonalA.Conflicts.Add(diagonalB.Id);
                            diagonalB.Conflicts.Add(diagonalA.Id);
                        }
                    }
                }
            }

            private DiagonalSelectionResult RunGreedySelection(List<CandidateDiagonal> candidateDiagonals, int strategy)
            {
                var availableIds = new HashSet<int>(candidateDiagonals.Select(candidate => candidate.Id));
                var usedBottomVerticals = new HashSet<int>();
                var usedTopVerticals = new HashSet<int>();
                var touchedVerticals = new HashSet<int>();
                var selected = new List<CandidateDiagonal>();

                while (availableIds.Count > 0)
                {
                    CandidateDiagonal bestCandidate = null;
                    int bestCompletedVerticals = int.MinValue;
                    int bestNewTouchedVerticals = int.MinValue;
                    int bestAvailableConflictCount = int.MaxValue;
                    double bestLength = double.MinValue;

                    foreach (int candidateId in availableIds)
                    {
                        CandidateDiagonal candidate = candidateDiagonals[candidateId];
                        if (usedBottomVerticals.Contains(candidate.BottomVerticalId) ||
                            usedTopVerticals.Contains(candidate.TopVerticalId))
                        {
                            continue;
                        }

                        int completedVerticals = 0;
                        if (usedTopVerticals.Contains(candidate.BottomVerticalId))
                        {
                            completedVerticals++;
                        }

                        if (usedBottomVerticals.Contains(candidate.TopVerticalId))
                        {
                            completedVerticals++;
                        }

                        int newTouchedVerticals = 0;
                        if (!touchedVerticals.Contains(candidate.BottomVerticalId))
                        {
                            newTouchedVerticals++;
                        }

                        if (!touchedVerticals.Contains(candidate.TopVerticalId))
                        {
                            newTouchedVerticals++;
                        }

                        int availableConflictCount = candidate.Conflicts.Count(conflictId => availableIds.Contains(conflictId));
                        double length = candidate.Line.Length;

                        bool isBetter = false;
                        if (bestCandidate == null)
                        {
                            isBetter = true;
                        }
                        else if (strategy == 0)
                        {
                            isBetter =
                                completedVerticals > bestCompletedVerticals ||
                                (completedVerticals == bestCompletedVerticals && newTouchedVerticals > bestNewTouchedVerticals) ||
                                (completedVerticals == bestCompletedVerticals && newTouchedVerticals == bestNewTouchedVerticals && availableConflictCount < bestAvailableConflictCount) ||
                                (completedVerticals == bestCompletedVerticals && newTouchedVerticals == bestNewTouchedVerticals && availableConflictCount == bestAvailableConflictCount && length > bestLength);
                        }
                        else if (strategy == 1)
                        {
                            isBetter =
                                newTouchedVerticals > bestNewTouchedVerticals ||
                                (newTouchedVerticals == bestNewTouchedVerticals && completedVerticals > bestCompletedVerticals) ||
                                (newTouchedVerticals == bestNewTouchedVerticals && completedVerticals == bestCompletedVerticals && availableConflictCount < bestAvailableConflictCount) ||
                                (newTouchedVerticals == bestNewTouchedVerticals && completedVerticals == bestCompletedVerticals && availableConflictCount == bestAvailableConflictCount && length > bestLength);
                        }
                        else
                        {
                            isBetter =
                                availableConflictCount < bestAvailableConflictCount ||
                                (availableConflictCount == bestAvailableConflictCount && completedVerticals > bestCompletedVerticals) ||
                                (availableConflictCount == bestAvailableConflictCount && completedVerticals == bestCompletedVerticals && newTouchedVerticals > bestNewTouchedVerticals) ||
                                (availableConflictCount == bestAvailableConflictCount && completedVerticals == bestCompletedVerticals && newTouchedVerticals == bestNewTouchedVerticals && length > bestLength);
                        }

                        if (!isBetter)
                        {
                            continue;
                        }

                        bestCandidate = candidate;
                        bestCompletedVerticals = completedVerticals;
                        bestNewTouchedVerticals = newTouchedVerticals;
                        bestAvailableConflictCount = availableConflictCount;
                        bestLength = length;
                    }

                    if (bestCandidate == null)
                    {
                        break;
                    }

                    selected.Add(bestCandidate);
                    usedBottomVerticals.Add(bestCandidate.BottomVerticalId);
                    usedTopVerticals.Add(bestCandidate.TopVerticalId);
                    touchedVerticals.Add(bestCandidate.BottomVerticalId);
                    touchedVerticals.Add(bestCandidate.TopVerticalId);

                    availableIds.Remove(bestCandidate.Id);
                    foreach (int conflictId in bestCandidate.Conflicts)
                    {
                        availableIds.Remove(conflictId);
                    }
                }

                return new DiagonalSelectionResult
                {
                    Selected = selected,
                    Metrics = EvaluateSelection(selected)
                };
            }

            private SelectionMetrics EvaluateSelection(List<CandidateDiagonal> selected)
            {
                return EvaluateSelection(selected, Enumerable.Empty<BandLayout>());
            }

            private SelectionMetrics EvaluateSelection(List<CandidateDiagonal> selected, IEnumerable<BandLayout> selectedLayouts)
            {
                var usedBottomVerticals = new HashSet<int>();
                var usedTopVerticals = new HashSet<int>();
                var touchedVerticals = new HashSet<int>();
                List<BandLayout> layoutList = selectedLayouts?.ToList() ?? new List<BandLayout>();

                foreach (CandidateDiagonal candidate in selected)
                {
                    usedBottomVerticals.Add(candidate.BottomVerticalId);
                    usedTopVerticals.Add(candidate.TopVerticalId);
                    touchedVerticals.Add(candidate.BottomVerticalId);
                    touchedVerticals.Add(candidate.TopVerticalId);
                }

                int fullyBracedVerticalCount = usedBottomVerticals.Count(verticalId => usedTopVerticals.Contains(verticalId));
                PathMetrics pathMetrics = EvaluatePathMetrics(selected);
                int normalizedForwardCount = layoutList.Count(layout => layout.NormalizedForwardPattern);
                int normalizedBackwardCount = layoutList.Count - normalizedForwardCount;
                return new SelectionMetrics
                {
                    LongestDiagonalPathCount = pathMetrics.LongestDiagonalPathCount,
                    LongestPathMemberCount = pathMetrics.LongestPathMemberCount,
                    PathComponentCount = pathMetrics.PathComponentCount,
                    FullyBracedVerticalCount = fullyBracedVerticalCount,
                    TouchedVerticalCount = touchedVerticals.Count,
                    DiagonalCount = selected.Count,
                    TotalPathDiagonalCount = pathMetrics.TotalPathDiagonalCount,
                    TotalPathMemberCount = pathMetrics.TotalPathMemberCount,
                    OrderedPathDiagonalCounts = new List<int>(pathMetrics.OrderedPathDiagonalCounts),
                    OrderedPathMemberCounts = new List<int>(pathMetrics.OrderedPathMemberCounts),
                    DominantOrientationBandCount = Math.Max(normalizedForwardCount, normalizedBackwardCount),
                    OpposingOrientationBandCount = Math.Min(normalizedForwardCount, normalizedBackwardCount)
                };
            }

            private HashSet<int> BuildTouchedVerticalSet(IEnumerable<CandidateDiagonal> selected)
            {
                var touchedVerticals = new HashSet<int>();

                foreach (CandidateDiagonal candidate in selected)
                {
                    touchedVerticals.Add(candidate.BottomVerticalId);
                    touchedVerticals.Add(candidate.TopVerticalId);
                }

                return touchedVerticals;
            }

            private int CountTouchedVerticals(IEnumerable<CandidateDiagonal> selected, HashSet<int> verticalFilter)
            {
                if (verticalFilter == null || verticalFilter.Count == 0)
                {
                    return 0;
                }

                HashSet<int> touchedVerticals = BuildTouchedVerticalSet(selected);
                return touchedVerticals.Count(verticalId => verticalFilter.Contains(verticalId));
            }

            private DiagonalSelectionResult ExtractBestUntouchedChain(
                List<CandidateDiagonal> diagonals,
                List<BandLayout> selectedLayouts,
                HashSet<int> untouchedVerticals)
            {
                PathComponent bestComponent = null;
                int bestUntouchedCount = 0;
                int bestUntouchedEndpointCount = 0;

                foreach (PathComponent component in BuildPathComponents(diagonals))
                {
                    int untouchedCount = CountTouchedVerticals(component.Diagonals, untouchedVerticals);
                    int untouchedEndpointCount = component.EndpointVerticalIds.Count(verticalId => untouchedVerticals.Contains(verticalId));
                    if (untouchedCount == 0 || untouchedEndpointCount == 0)
                    {
                        continue;
                    }

                    bool isBetter =
                        bestComponent == null ||
                        untouchedEndpointCount > bestUntouchedEndpointCount ||
                        (untouchedEndpointCount == bestUntouchedEndpointCount && component.Diagonals.Count > bestComponent.Diagonals.Count) ||
                        (untouchedEndpointCount == bestUntouchedEndpointCount && component.Diagonals.Count == bestComponent.Diagonals.Count && component.MemberCount > bestComponent.MemberCount) ||
                        (untouchedEndpointCount == bestUntouchedEndpointCount && component.Diagonals.Count == bestComponent.Diagonals.Count && component.MemberCount == bestComponent.MemberCount && untouchedCount > bestUntouchedCount) ||
                        (untouchedEndpointCount == bestUntouchedEndpointCount && component.Diagonals.Count == bestComponent.Diagonals.Count && component.MemberCount == bestComponent.MemberCount && untouchedCount == bestUntouchedCount && component.LayoutIds.Count < bestComponent.LayoutIds.Count);

                    if (!isBetter)
                    {
                        continue;
                    }

                    bestComponent = component;
                    bestUntouchedCount = untouchedCount;
                    bestUntouchedEndpointCount = untouchedEndpointCount;
                }

                if (bestComponent == null)
                {
                    return new DiagonalSelectionResult();
                }

                List<BandLayout> componentLayouts = selectedLayouts
                    .Where(layout => bestComponent.LayoutIds.Contains(layout.LayoutId))
                    .ToList();

                return CreateSelectionResult(bestComponent.Diagonals, componentLayouts);
            }

            private sealed class PathComponent
            {
                public List<CandidateDiagonal> Diagonals = new List<CandidateDiagonal>();
                public HashSet<int> LayoutIds = new HashSet<int>();
                public HashSet<int> EndpointVerticalIds = new HashSet<int>();
                public int MemberCount;
            }

            private sealed class PathMetrics
            {
                public int LongestDiagonalPathCount;
                public int LongestPathMemberCount;
                public int PathComponentCount;
                public int TotalPathDiagonalCount;
                public int TotalPathMemberCount;
                public List<int> OrderedPathDiagonalCounts = new List<int>();
                public List<int> OrderedPathMemberCounts = new List<int>();
            }

            private List<PathComponent> BuildPathComponents(List<CandidateDiagonal> selected)
            {
                if (selected.Count == 0)
                {
                    return new List<PathComponent>();
                }

                var adjacency = new Dictionary<int, List<int>>();
                for (int index = 0; index < selected.Count; index++)
                {
                    adjacency[index] = new List<int>();
                }

                for (int indexA = 0; indexA < selected.Count; indexA++)
                {
                    CandidateDiagonal diagonalA = selected[indexA];

                    for (int indexB = indexA + 1; indexB < selected.Count; indexB++)
                    {
                        CandidateDiagonal diagonalB = selected[indexB];
                        if (!ConnectsThroughVertical(diagonalA, diagonalB))
                        {
                            continue;
                        }

                        adjacency[indexA].Add(indexB);
                        adjacency[indexB].Add(indexA);
                    }
                }

                var visited = new HashSet<int>();
                var components = new List<PathComponent>();

                for (int index = 0; index < selected.Count; index++)
                {
                    if (!visited.Add(index))
                    {
                        continue;
                    }

                    var stack = new Stack<int>();
                    stack.Push(index);
                    var component = new PathComponent();

                    int edgeAccumulator = 0;

                    while (stack.Count > 0)
                    {
                        int current = stack.Pop();
                        CandidateDiagonal currentDiagonal = selected[current];
                        component.Diagonals.Add(currentDiagonal);
                        component.LayoutIds.Add(currentDiagonal.LayoutId);
                        edgeAccumulator += adjacency[current].Count;

                        foreach (int neighbor in adjacency[current])
                        {
                            if (visited.Add(neighbor))
                            {
                                stack.Push(neighbor);
                            }
                        }
                    }

                    int edgeCount = edgeAccumulator / 2;
                    component.MemberCount = component.Diagonals.Count + edgeCount;
                    PopulateEndpointVerticals(component);
                    components.Add(component);
                }

                return components;
            }

            private void PopulateEndpointVerticals(PathComponent component)
            {
                var bottomVerticals = new HashSet<int>(component.Diagonals.Select(diagonal => diagonal.BottomVerticalId));
                var topVerticals = new HashSet<int>(component.Diagonals.Select(diagonal => diagonal.TopVerticalId));

                foreach (int verticalId in bottomVerticals)
                {
                    if (!topVerticals.Contains(verticalId))
                    {
                        component.EndpointVerticalIds.Add(verticalId);
                    }
                }

                foreach (int verticalId in topVerticals)
                {
                    if (!bottomVerticals.Contains(verticalId))
                    {
                        component.EndpointVerticalIds.Add(verticalId);
                    }
                }
            }

            private PathMetrics EvaluatePathMetrics(List<CandidateDiagonal> selected)
            {
                var metrics = new PathMetrics();

                foreach (PathComponent component in BuildPathComponents(selected))
                {
                    int diagonalCount = component.Diagonals.Count;
                    int memberCount = component.MemberCount;

                    metrics.PathComponentCount++;
                    metrics.TotalPathDiagonalCount += diagonalCount;
                    metrics.TotalPathMemberCount += memberCount;
                    metrics.OrderedPathDiagonalCounts.Add(diagonalCount);
                    metrics.OrderedPathMemberCounts.Add(memberCount);
                    metrics.LongestDiagonalPathCount = Math.Max(metrics.LongestDiagonalPathCount, diagonalCount);
                    metrics.LongestPathMemberCount = Math.Max(metrics.LongestPathMemberCount, memberCount);
                }

                metrics.OrderedPathDiagonalCounts.Sort((left, right) => right.CompareTo(left));
                metrics.OrderedPathMemberCounts.Sort((left, right) => right.CompareTo(left));
                return metrics;
            }

            private bool ConnectsThroughVertical(CandidateDiagonal diagonalA, CandidateDiagonal diagonalB)
            {
                return diagonalA.TopVerticalId == diagonalB.BottomVerticalId ||
                       diagonalA.BottomVerticalId == diagonalB.TopVerticalId;
            }

            private bool DiagonalsIntersectInInterior(CandidateDiagonal diagonalA, CandidateDiagonal diagonalB)
            {
                if (SharesEndpointNode(diagonalA, diagonalB))
                {
                    return false;
                }

                bool intersects = Rhino.Geometry.Intersect.Intersection.LineLine(
                    diagonalA.Line,
                    diagonalB.Line,
                    out double parameterA,
                    out double parameterB,
                    _tolerance,
                    true);

                if (!intersects)
                {
                    return false;
                }

                Point3d pointA = diagonalA.Line.PointAt(parameterA);
                Point3d pointB = diagonalB.Line.PointAt(parameterB);
                if (pointA.DistanceToSquared(pointB) > _toleranceSquared)
                {
                    return false;
                }

                return !IsEndpointParameter(parameterA) && !IsEndpointParameter(parameterB);
            }

            private bool SharesEndpointNode(CandidateDiagonal diagonalA, CandidateDiagonal diagonalB)
            {
                return diagonalA.BottomNodeId == diagonalB.BottomNodeId ||
                       diagonalA.BottomNodeId == diagonalB.TopNodeId ||
                       diagonalA.TopNodeId == diagonalB.BottomNodeId ||
                       diagonalA.TopNodeId == diagonalB.TopNodeId;
            }

            private bool IsEndpointParameter(double parameter)
            {
                return parameter <= 1e-6 || parameter >= 1.0 - 1e-6;
            }

            private List<BraceBand> BuildBraceBands(List<CandidateFace> candidateFaces)
            {
                var faceLookup = candidateFaces.ToDictionary(face => face.Id);
                var faceAdjacency = candidateFaces.ToDictionary(face => face.Id, face => new List<FaceNeighbor>());
                var facesByVertical = new Dictionary<int, List<int>>();

                foreach (CandidateFace face in candidateFaces)
                {
                    AddFaceToVerticalBucket(face.VerticalA.Id, face.Id, facesByVertical);
                    AddFaceToVerticalBucket(face.VerticalB.Id, face.Id, facesByVertical);
                }

                foreach (KeyValuePair<int, List<int>> bucket in facesByVertical)
                {
                    List<int> faceIds = bucket.Value;
                    for (int indexA = 0; indexA < faceIds.Count; indexA++)
                    {
                        for (int indexB = indexA + 1; indexB < faceIds.Count; indexB++)
                        {
                            CandidateFace faceA = faceLookup[faceIds[indexA]];
                            CandidateFace faceB = faceLookup[faceIds[indexB]];
                            if (!AreDirectionsCompatible(faceA.Direction, faceB.Direction))
                            {
                                continue;
                            }

                            faceAdjacency[faceA.Id].Add(new FaceNeighbor
                            {
                                NeighborFaceId = faceB.Id,
                                SharedVerticalId = bucket.Key
                            });

                            faceAdjacency[faceB.Id].Add(new FaceNeighbor
                            {
                                NeighborFaceId = faceA.Id,
                                SharedVerticalId = bucket.Key
                            });
                        }
                    }
                }

                var unassignedFaces = new HashSet<int>(candidateFaces.Select(face => face.Id));
                var bands = new List<BraceBand>();
                int nextBandId = 0;

                while (unassignedFaces.Count > 0)
                {
                    int startFaceId = unassignedFaces
                        .OrderBy(faceId => CountUnassignedNeighbors(faceId, faceAdjacency, unassignedFaces))
                        .ThenBy(faceId => faceId)
                        .First();

                    var orderedFaces = new List<OrientedFace>();
                    int? previousSharedVerticalId = null;
                    int previousFaceId = -1;
                    int currentFaceId = startFaceId;

                    while (currentFaceId >= 0)
                    {
                        List<FaceNeighbor> nextCandidates = faceAdjacency[currentFaceId]
                            .Where(neighbor => unassignedFaces.Contains(neighbor.NeighborFaceId) && neighbor.NeighborFaceId != previousFaceId)
                            .OrderBy(neighbor => CountUnassignedNeighbors(neighbor.NeighborFaceId, faceAdjacency, unassignedFaces))
                            .ThenBy(neighbor => neighbor.NeighborFaceId)
                            .ToList();

                        int? nextSharedVerticalId = nextCandidates.Count > 0 ? nextCandidates[0].SharedVerticalId : (int?)null;
                        int nextFaceId = nextCandidates.Count > 0 ? nextCandidates[0].NeighborFaceId : -1;

                        orderedFaces.Add(OrientFace(faceLookup[currentFaceId], previousSharedVerticalId, nextSharedVerticalId));
                        unassignedFaces.Remove(currentFaceId);

                        previousFaceId = currentFaceId;
                        previousSharedVerticalId = nextSharedVerticalId;
                        currentFaceId = nextFaceId;
                    }

                    if (orderedFaces.Count == 0)
                    {
                        continue;
                    }

                    bands.Add(new BraceBand
                    {
                        Id = nextBandId++,
                        Direction = AverageDirection(orderedFaces.Select(item => item.Face.Direction)),
                        Faces = orderedFaces
                    });
                }

                return bands;
            }

            private List<BraceBand> SelectDominantDirectionBands(List<BraceBand> bands)
            {
                var directionFamilies = new List<List<BraceBand>>();

                foreach (BraceBand band in bands)
                {
                    bool placed = false;

                    foreach (List<BraceBand> family in directionFamilies)
                    {
                        if (AreDirectionsCompatible(family[0].Direction, band.Direction))
                        {
                            family.Add(band);
                            placed = true;
                            break;
                        }
                    }

                    if (!placed)
                    {
                        directionFamilies.Add(new List<BraceBand> { band });
                    }
                }

                if (directionFamilies.Count == 0)
                {
                    return bands;
                }

                return directionFamilies
                    .OrderByDescending(family => family.Sum(item => item.Faces.Count))
                    .ThenByDescending(family => family.Count)
                    .First();
            }

            private int ScoreBandPattern(BraceBand band, bool useForwardPattern, HashSet<int> usedAngledNodes)
            {
                int score = 0;

                foreach (OrientedFace face in band.Faces)
                {
                    int lowNodeId = useForwardPattern ? face.LowerStartNodeId : face.LowerEndNodeId;
                    int highNodeId = useForwardPattern ? face.UpperEndNodeId : face.UpperStartNodeId;

                    if (usedAngledNodes.Contains(lowNodeId) || usedAngledNodes.Contains(highNodeId))
                    {
                        continue;
                    }

                    score++;
                }

                return score;
            }

            private void AddBandPattern(BraceBand band, bool useForwardPattern, HashSet<int> usedAngledNodes)
            {
                foreach (OrientedFace face in band.Faces)
                {
                    int lowNodeId = useForwardPattern ? face.LowerStartNodeId : face.LowerEndNodeId;
                    int highNodeId = useForwardPattern ? face.UpperEndNodeId : face.UpperStartNodeId;

                    if (usedAngledNodes.Contains(lowNodeId) || usedAngledNodes.Contains(highNodeId))
                    {
                        continue;
                    }

                    if (AddMember(new Line(Nodes[lowNodeId].Point, Nodes[highNodeId].Point), MemberType.Angled) == null)
                    {
                        continue;
                    }

                    usedAngledNodes.Add(lowNodeId);
                    usedAngledNodes.Add(highNodeId);
                    BracedFaceCount++;
                }
            }

            private OrientedFace OrientFace(CandidateFace face, int? previousSharedVerticalId, int? nextSharedVerticalId)
            {
                bool useNaturalOrder = true;

                if (previousSharedVerticalId.HasValue && nextSharedVerticalId.HasValue)
                {
                    if (face.VerticalA.Id == previousSharedVerticalId.Value && face.VerticalB.Id == nextSharedVerticalId.Value)
                    {
                        useNaturalOrder = true;
                    }
                    else if (face.VerticalB.Id == previousSharedVerticalId.Value && face.VerticalA.Id == nextSharedVerticalId.Value)
                    {
                        useNaturalOrder = false;
                    }
                }
                else if (previousSharedVerticalId.HasValue)
                {
                    if (face.VerticalA.Id == previousSharedVerticalId.Value)
                    {
                        useNaturalOrder = true;
                    }
                    else if (face.VerticalB.Id == previousSharedVerticalId.Value)
                    {
                        useNaturalOrder = false;
                    }
                }
                else if (nextSharedVerticalId.HasValue)
                {
                    if (face.VerticalB.Id == nextSharedVerticalId.Value)
                    {
                        useNaturalOrder = true;
                    }
                    else if (face.VerticalA.Id == nextSharedVerticalId.Value)
                    {
                        useNaturalOrder = false;
                    }
                }

                GraphEdge leftVertical = useNaturalOrder ? face.VerticalA : face.VerticalB;
                GraphEdge rightVertical = useNaturalOrder ? face.VerticalB : face.VerticalA;

                return new OrientedFace
                {
                    Face = face,
                    LeftVerticalId = leftVertical.Id,
                    RightVerticalId = rightVertical.Id,
                    LowerStartNodeId = leftVertical.LowNodeId,
                    LowerEndNodeId = rightVertical.LowNodeId,
                    UpperStartNodeId = leftVertical.HighNodeId,
                    UpperEndNodeId = rightVertical.HighNodeId
                };
            }

            private static void AddFaceToVerticalBucket(int verticalId, int faceId, Dictionary<int, List<int>> facesByVertical)
            {
                if (!facesByVertical.TryGetValue(verticalId, out List<int> faceIds))
                {
                    faceIds = new List<int>();
                    facesByVertical[verticalId] = faceIds;
                }

                faceIds.Add(faceId);
            }

            private static int CountUnassignedNeighbors(int faceId, Dictionary<int, List<FaceNeighbor>> faceAdjacency, HashSet<int> unassignedFaces)
            {
                return faceAdjacency[faceId].Count(neighbor => unassignedFaces.Contains(neighbor.NeighborFaceId));
            }

            private static bool AreDirectionsCompatible(Vector3d directionA, Vector3d directionB)
            {
                Vector3d unitA = directionA;
                Vector3d unitB = directionB;

                if (!unitA.Unitize() || !unitB.Unitize())
                {
                    return false;
                }

                return Math.Abs(unitA * unitB) >= 0.7;
            }

            private static Vector3d AverageDirection(IEnumerable<Vector3d> directions)
            {
                Vector3d average = Vector3d.Zero;

                foreach (Vector3d rawDirection in directions)
                {
                    Vector3d direction = rawDirection;
                    if (!direction.Unitize())
                    {
                        continue;
                    }

                    if (!average.IsTiny() && (average * direction) < 0.0)
                    {
                        direction = -direction;
                    }

                    average += direction;
                }

                if (average.IsTiny())
                {
                    return Vector3d.Unset;
                }

                average.Unitize();
                return average;
            }

            private static Vector3d AverageDirection(Vector3d directionA, Vector3d directionB)
            {
                return AverageDirection(new[] { directionA, directionB });
            }

            private GraphEdge AddMember(Line line, MemberType type)
            {
                if (!line.IsValid || line.Length <= _tolerance)
                {
                    return null;
                }

                int a = GetOrCreateNode(line.From);
                int b = GetOrCreateNode(line.To);
                if (a == b)
                {
                    return null;
                }

                string key = BuildTypedMemberKey(a, b, type);
                if (!_memberKeys.Add(key))
                {
                    return null;
                }

                Point3d from = Nodes[a].Point;
                Point3d to = Nodes[b].Point;
                var mergedLine = new Line(from, to);

                int lowNodeId = a;
                int highNodeId = b;
                if (from.Z > to.Z)
                {
                    lowNodeId = b;
                    highNodeId = a;
                }

                int nextEdgeId = HorizontalEdges.Count + VerticalEdges.Count + AngledEdges.Count;

                var edge = new GraphEdge
                {
                    Id = nextEdgeId,
                    A = a,
                    B = b,
                    LowNodeId = lowNodeId,
                    HighNodeId = highNodeId,
                    Line = mergedLine,
                    Type = type
                };

                if (type == MemberType.Horizontal)
                {
                    HorizontalEdges.Add(edge);
                }
                else if (type == MemberType.Vertical)
                {
                    VerticalEdges.Add(edge);
                }
                else
                {
                    AngledEdges.Add(edge);
                }

                return edge;
            }

            private IEnumerable<Line> ExtractSegments(Curve curve)
            {
                if (curve == null || !curve.IsValid)
                {
                    yield break;
                }

                if (curve.TryGetPolyline(out Polyline polyline) && polyline.Count >= 2)
                {
                    foreach (Line line in PolylineToSegments(polyline))
                    {
                        yield return line;
                    }

                    yield break;
                }

                Curve[] duplicatedSegments = curve.DuplicateSegments();
                if (duplicatedSegments != null && duplicatedSegments.Length > 0)
                {
                    foreach (Curve segment in duplicatedSegments)
                    {
                        if (segment == null)
                        {
                            continue;
                        }

                        if (segment.TryGetPolyline(out Polyline segmentPolyline) && segmentPolyline.Count >= 2)
                        {
                            foreach (Line line in PolylineToSegments(segmentPolyline))
                            {
                                yield return line;
                            }
                        }
                        else if (segment.IsLinear(_tolerance))
                        {
                            yield return new Line(segment.PointAtStart, segment.PointAtEnd);
                        }
                    }

                    yield break;
                }

                if (curve.IsLinear(_tolerance))
                {
                    yield return new Line(curve.PointAtStart, curve.PointAtEnd);
                }
            }

            private IEnumerable<Line> PolylineToSegments(Polyline polyline)
            {
                for (int index = 0; index < polyline.Count - 1; index++)
                {
                    Point3d from = polyline[index];
                    Point3d to = polyline[index + 1];
                    if (from.DistanceToSquared(to) <= _toleranceSquared)
                    {
                        continue;
                    }

                    yield return new Line(from, to);
                }
            }

            private MemberType ClassifySegment(Line line)
            {
                Vector3d delta = line.To - line.From;
                double xyLength = Math.Sqrt((delta.X * delta.X) + (delta.Y * delta.Y));
                double zLength = Math.Abs(delta.Z);

                if (xyLength <= _tolerance && zLength > _tolerance)
                {
                    return MemberType.Vertical;
                }

                if (zLength <= _tolerance)
                {
                    return MemberType.Horizontal;
                }

                return MemberType.Angled;
            }

            private int GetOrCreateNode(Point3d point)
            {
                Tuple<int, int, int> baseKey = BucketKey(point);

                for (int x = -1; x <= 1; x++)
                {
                    for (int y = -1; y <= 1; y++)
                    {
                        for (int z = -1; z <= 1; z++)
                        {
                            var neighborKey = Tuple.Create(baseKey.Item1 + x, baseKey.Item2 + y, baseKey.Item3 + z);
                            if (!_nodeBuckets.TryGetValue(neighborKey, out List<int> candidates))
                            {
                                continue;
                            }

                            foreach (int nodeId in candidates)
                            {
                                if (Nodes[nodeId].Point.DistanceToSquared(point) <= _toleranceSquared)
                                {
                                    return nodeId;
                                }
                            }
                        }
                    }
                }

                int newId = Nodes.Count;
                Nodes.Add(new GraphNode
                {
                    Id = newId,
                    Point = point
                });

                if (!_nodeBuckets.TryGetValue(baseKey, out List<int> bucket))
                {
                    bucket = new List<int>();
                    _nodeBuckets[baseKey] = bucket;
                }

                bucket.Add(newId);
                return newId;
            }

            private Tuple<int, int, int> BucketKey(Point3d point)
            {
                return Tuple.Create(
                    (int)Math.Floor(point.X / _tolerance),
                    (int)Math.Floor(point.Y / _tolerance),
                    (int)Math.Floor(point.Z / _tolerance));
            }

            private static string BuildTypedMemberKey(int a, int b, MemberType type)
            {
                int min = Math.Min(a, b);
                int max = Math.Max(a, b);
                return ((int)type).ToString() + ":" + min + "-" + max;
            }

            private static string BuildNodePairKey(int a, int b)
            {
                int min = Math.Min(a, b);
                int max = Math.Max(a, b);
                return min + "-" + max;
            }

            private static string BuildFaceKey(int lowA, int lowB, int highA, int highB)
            {
                string lowerKey = BuildNodePairKey(lowA, lowB);
                string upperKey = BuildNodePairKey(highA, highB);
                return lowerKey + "|" + upperKey;
            }
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            var gridCurves = new List<Curve>();
            var interLayerStruts = new List<Line>();
            double tolerance = RhinoDoc.ActiveDoc?.ModelAbsoluteTolerance ?? 0.01;

            if (!DA.GetDataList(0, gridCurves))
            {
                return;
            }

            DA.GetData(1, ref tolerance);
            DA.GetDataList(2, interLayerStruts);

            if (gridCurves.Count == 0 && interLayerStruts.Count == 0)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "No geometry was supplied.");
                return;
            }

            var graph = new BracedGraph(tolerance);
            bool hasExplicitStruts = interLayerStruts.Count > 0;

            if (hasExplicitStruts)
            {
                foreach (Curve gridCurve in gridCurves)
                {
                    graph.AddHorizontalCurve(gridCurve);
                }

                foreach (Line strut in interLayerStruts)
                {
                    graph.AddVerticalLine(strut);
                }
            }
            else
            {
                foreach (Curve pathCurve in gridCurves)
                {
                    graph.AddFallbackCurve(pathCurve);
                }
            }

            graph.BuildSingleBraceBands();

            DataTree<Line> allMembersTree = ToLineTree(graph.AllEdges.Select(edge => edge.Line));
            DataTree<Point3d> nodesTree = ToPointTree(graph.Nodes.Select(node => node.Point));
            DataTree<Line> horizontalTree = ToLineTree(graph.HorizontalEdges.Select(edge => edge.Line));
            DataTree<Line> verticalTree = ToLineTree(graph.VerticalEdges.Select(edge => edge.Line));
            DataTree<Line> angledTree = ToLineTree(graph.AngledEdges.Select(edge => edge.Line));

            string analysis = string.Join(Environment.NewLine, new[]
            {
                "Graph nodes: " + graph.Nodes.Count,
                "Horizontal members: " + graph.HorizontalEdges.Count,
                "Vertical members: " + graph.VerticalEdges.Count,
                "Angled members: " + graph.AngledEdges.Count,
                "Optimized braced faces: " + graph.BracedFaceCount,
                "Longest diagonal path length: " + graph.LongestDiagonalPathCount,
                "Longest path member count: " + graph.LongestPathMemberCount,
                "Verticals touched by angled members: " + graph.TouchedVerticalCount,
                "Verticals fully braced at top and bottom: " + graph.FullyBracedVerticalCount,
                "Optimization priority: longest primary chain first, then supplemental chains for untouched verticals, then dominant layer direction as a tie-break.",
                hasExplicitStruts
                    ? "Classification mode: explicit warped-grid semantics (grid curves = horizontal, inter-layer struts = vertical)."
                    : "Classification mode: inferred from geometry because no explicit inter-layer struts were supplied."
            });

            DA.SetDataTree(0, allMembersTree);
            DA.SetDataTree(1, nodesTree);
            DA.SetDataTree(2, horizontalTree);
            DA.SetDataTree(3, verticalTree);
            DA.SetDataTree(4, angledTree);
            DA.SetData(5, analysis);
        }

        private static DataTree<Line> ToLineTree(IEnumerable<Line> lines)
        {
            var tree = new DataTree<Line>();
            GH_Path path = new GH_Path(0);
            foreach (Line line in lines)
            {
                tree.Add(line, path);
            }

            return tree;
        }

        private static DataTree<Point3d> ToPointTree(IEnumerable<Point3d> points)
        {
            var tree = new DataTree<Point3d>();
            GH_Path path = new GH_Path(0);
            foreach (Point3d point in points)
            {
                tree.Add(point, path);
            }

            return tree;
        }

        protected override System.Drawing.Bitmap Icon => null;

        public override Guid ComponentGuid
        {
            get { return new Guid("62b75d2c-a076-4dc3-bd68-4950444aad36"); }
        }
    }
}
