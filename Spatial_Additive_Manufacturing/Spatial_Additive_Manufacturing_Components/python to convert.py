import rhinoscriptsyntax as rs
import scriptcontext as sc

import System
import System.Collections.Generic
import Rhino
from operator import itemgetter

import heapq
import time

# list of points and lines as your graph representation
# For example:
lines = crvs
points = nodes

graphLines = []
graphPoints = []

for line in lines:
    pt1 = line.From
    pt2 = line.To
    graphLines.append(Rhino.Geometry.Line(pt1, pt2))

for point in points:
    tempPt = point

    graphPoints.append(Rhino.Geometry.Point3d(tempPt.X, tempPt.Y, tempPt.Z))

def add_weight_to_lines(graph):
    """
    Assign a weight to each line based on the average Z height of its start and end points.

    Parameters:
    lines (list): List of Rhino.Geometry.Line objects.

    Returns:
    dict: A dictionary mapping each line to its assigned weight.
    """
    
    if not lines:
        raise ValueError("Input list of lines is empty")

    weights = {}
    for line in lines:
        start_point = line.From
        end_point = line.To


        weight = 0
        # Calculate the average Z height of start and end points

        average_z = (start_point.Z + end_point.Z) / 2


        #print(start_point.Z, end_point.Z)

        # define if the curve is an angled, horizontal, or angled so weights can be assigned properly
        if start_point.Z == end_point.Z:
            #print("Horizontal Line")
            pass

        elif round(start_point.X, 2) == round(end_point.X, 2) and round(start_point.Y, 2) == round(end_point.Y, 2):
            #print("Vertical Line", "X:", round(start_point.X, 2), round(end_point.X, 2), "Y:", round(start_point.Y, 2), round(end_point.Y, 2))
            # Determine the highest Z point
            if start_point.Z > end_point.Z:
                line.Flip()
                
            #define function that will find angled intersections 

            weight_vertical_at_start = find_intersection_vertical_at_end(line, lines)
            weight_vertical_at_end = find_intersection_vertical_at_start(line, lines)
            weight_vertical_no_top = vertical_no_angle_at_top(line, lines)
            

            weight = weight + weight_vertical_at_start + weight_vertical_at_end + weight_vertical_no_top
           
        else:
            #print("Angled Line",  "X:", round(start_point.X, 2), round(end_point.X, 2), "Y:", round(start_point.Y, 2), round(end_point.Y, 2))
            if start_point.Z > end_point.Z:
                line.Flip()
            #define function that will find Vertical endpoint intersections
            weight_angled_atstart = find_intersection_angled_at_start(line, lines)
            weight_angled_atend = find_intersection_angled_at_end(line, lines)
            weight_with_angled_at_start = verticals_with_angled_at_start(line, lines)
            

            weight = weight + weight_angled_atstart + weight_angled_atend + weight_with_angled_at_start
            
        #visualize current line
        midpoint = line.PointAtLength(line.Length/2)

        
        # Assign weight based on average Z height
        weight_z = calculate_weight(average_z)
        
        #text_dot = rs.AddTextDot(round(weight, 3), midpoint)
    
        # Store the weight in the dictionary
        weight = weight + weight_z - 32.481
        weight = round(weight, 3)
        weights[line] = weight
        #rs.DeleteObject(text_dot)
    return weights

def find_intersection_vertical_at_start(input_line, graph_lines):
    #find the intersection for a vertical line
    start_point = input_line.From
    end_point = input_line.To


    weight_vertical = 0

    if start_point.Z > end_point.Z:
        input_line.Flip()
    
    for line in graph_lines:

        if abs(line.To.Z - line.From.Z) <= .02:
            #print("Horizontal Line")
            weight_vertical = 0
            
        elif line.From.X == line.To.X and line.From.Y == line.To.Y:
            weight_vertical = 0
        
        else:
            if line.From.Z > line.To.Z:
                line.Flip()
            #intersection at the startpoint
            if round(line.From.X, 2) == round(start_point.X, 2) and round(line.From.Y, 2) == round(start_point.Y, 2) and round(line.From.Z, 2) == round(start_point.Z, 2):
                weight_vertical = 0.1
                return weight_vertical            

    
    return weight_vertical 

def find_intersection_vertical_at_end(input_line, graph_lines):
    #find the intersection for a vertical line
    start_point = input_line.From
    end_point = input_line.To


    weight_vertical = 0

    if start_point.Z > end_point.Z:
        input_line.Flip()
    
    for line in graph_lines:
        #thirdpoint = line.PointAtLength(line.Length/3)
        #graph_line_dot = rs.AddTextDot("graph_line", thirdpoint)

        if abs(end_point.Z - start_point.Z) <= .02:
            #print("Horizontal Line")
            
            weight_vertical = 0

        elif line.From.X == line.To.X and line.From.Y == line.To.Y:
            #print("Vertical Line")
            weight_vertical = 0
        else:
            if line.From.Z > line.To.Z:
                line.Flip()
                #intersection at the endpoint
            if round(line.To.X, 2) == round(end_point.X, 2) and round(line.To.Y, 2) == round(end_point.Y, 2) and round(line.To.Z, 2) == round(end_point.Z, 2):
                weight_vertical = 0.05
                return weight_vertical            

    return weight_vertical 

def find_intersection_angled_at_start(input_line, graph_lines):
    #find the intersection for a vertical line, inputline is vertical
    start_point = input_line.From
    end_point = input_line.To
    #otherpoint = input_line.PointAtLength(input_line.Length/4)
    #current_dot = rs.AddTextDot("input_line", otherpoint)

    weight_angled = 0

    if start_point.Z > end_point.Z:
        input_line.Flip()
    
    for line in graph_lines:
        #thirdpoint = line.PointAtLength(line.Length/3)
        #graph_line_dot = rs.AddTextDot("graph_line", thirdpoint)
        if abs(end_point.Z - start_point.Z) <= .02:
            #print("Horizontal Line")
            #rs.DeleteObject(graph_line_dot)
            weight_angled = 0
        elif round(line.From.X, 2) == round(line.To.X, 2) and round(line.From.Y, 2) == round(line.To.Y, 2):
            #print("Vertical Line")
            if line.From.Z > line.To.Z:
                line.Flip()
            #intersection at the startpoint
            if round(line.From.X, 2) == round(start_point.X, 2) and round(line.From.Y, 2) == round(start_point.Y, 2) and round(line.From.Z, 2) == round(start_point.Z, 2):
                weight_angled = -0.15
                return weight_angled

        else:
            weight_angled = 0
  
    return weight_angled

def find_intersection_angled_at_end(input_line, graph_lines):
    #find the intersection for a vertical line
    start_point = input_line.From
    end_point = input_line.To
    #otherpoint = input_line.PointAtLength(input_line.Length/4)
    #current_dot = rs.AddTextDot("input_line", otherpoint)

    weight_angled = 0

    if start_point.Z > end_point.Z:
        input_line.Flip()
    
    for line in graph_lines:
        #thirdpoint = line.PointAtLength(line.Length/3)
        #graph_line_dot = rs.AddTextDot("graph_line", thirdpoint)

        if abs(line.To.Z - line.From.Z) <= .02:
            #print("Horizontal Line")
            #rs.DeleteObject(graph_line_dot)
            weight_angled = 0

        elif line.From.X == line.To.X and line.From.Y == line.To.Y:
            #print("Vertical Line")
            if line.From.Z > line.To.Z:
                line.Flip()

            #intersection at the endpoint
            if round(line.To.X, 2) == round(end_point.X, 2) and round(line.To.Y, 2) == round(end_point.Y, 2) and round(line.To.Z, 2) == round(end_point.Z, 2):
                weight_angled = 0.21
                return weight_angled 

        else:
            weight_angled = 0

    return weight_angled

def verticals_with_angled_at_start(input_line, graph_lines):
    #find the intersection for a vertical line
    start_point = input_line.From
    end_point = input_line.To
    #otherpoint = input_line.PointAtLength(input_line.Length/4)
    #current_dot = rs.AddTextDot("input_line", otherpoint)

    weight_angled = 0
    
    if start_point.Z > end_point.Z:
        input_line.Flip()
    #test if line connected to input line is vertical and attached to the end at both lines
    for line in graph_lines:
        linepoint = line.PointAtLength(line.Length/2)
        #test_dot = rs.AddTextDot("test_line", linepoint)
        if line.From.Z > line.To.Z:
            line.Flip()
        #test if the same line
        if round(line.From.X, 2) == round(start_point.X, 2) and round(line.From.Y, 2) == round(start_point.Y, 2) and round(line.From.Z, 2) == round(start_point.Z, 2) and round(line.To.X, 2) == round(end_point.X, 2) and round(line.To.Y, 2) == round(end_point.Y, 2) and round(line.To.Z, 2) == round(end_point.Z, 2):
            pass
        #test if same endpoint 
        elif round(end_point.X, 2) == round(line.To.X, 2) and round(end_point.Y, 2) == round(line.To.Y, 2) and round(end_point.Z, 2) == round(line.To.Z, 2):
                if round(line.From.X, 2) == round(line.To.X, 2) and round(line.From.Y, 2) == round(line.To.Y, 2):
                    weight_angled = find_intersection_vertical_at_start(line, graph_lines)
                    #rs.DeleteObject(current_dot)
                    #rs.DeleteObject(test_dot)
                    return weight_angled

        #rs.DeleteObject(test_dot)
    #rs.DeleteObject(current_dot)
    #rs.DeleteObject(test_dot)




    return weight_angled

#define a vertical line with no angled attached to the top where
#the angled average Z is lower than or equal to the vertical line 
def vertical_no_angle_at_top(input_line, graph_lines):
    start_point = input_line.From
    end_point = input_line.To


    average_z_input_line = (start_point.Z + end_point.Z) / 2
    weight_vertical = 0
    \
    if start_point.Z > end_point.Z:
        input_line.Flip()
    count = 0
    for line in graph_lines:


        if abs(line.To.Z - line.From.Z) <= .02:
            #print("Horizontal Line")
            weight_vertical = 0
            
        elif line.From.X == line.To.X and line.From.Y == line.To.Y:
            weight_vertical = 0
        
        else:
            if line.From.Z > line.To.Z:
                line.Flip()
            average_z_line = (line.From.Z + line.To.Z) / 2
            #intersection at the startpoint
            if average_z_line <= average_z_input_line:
                if round(line.To.X, 2) == round(end_point.X, 2) and round(line.To.Y, 2) == round(end_point.Y, 2) and round(line.To.Z, 2) == round(end_point.Z, 2):
                    count += 1

    if count == 0:
        weight_vertical = .07
    
    return weight_vertical 

def calculate_weight(average_z):
    """

    Parameters:
    average_z (float): Average Z height.

    Returns:
    float: Assigned weight.
    """

    weight = average_z # Adjust the multiplier as needed
    return weight 



# Function to change line color and add to "visited" layer
def mark_line_as_visited(line, order):
    rs.ObjectColor(line, [255, 0, 0])  
    rs.ObjectLayer(line, visited_layer_name)
    # Add a text dot with the order number
    #midpoint = rs.CurveMidPoint(line)
    #text_dot = rs.AddTextDot(str(order), midpoint)
    time.sleep(0.25)




# points = [Rhino.Geometry.Point3d(x, y, z) for x, y, z in point_coordinates]
# lines = [Rhino.Geometry.Line(pt1, pt2) for pt1, pt2 in line_endpoints]

# Create a graph using points and lines
graph = {i: [] for i in range(len(graphPoints))}
for line in graphLines:
    start, end = line.From, line.To
    if start in graphPoints and end in graphPoints:
        graph[graphPoints.index(start)].append(graphPoints.index(end))
        graph[graphPoints.index(end)].append(graphPoints.index(start))
    else:
        print("Start or end point not found in graphPoints.")


# DFS function
def dfs_all_lines():
    visited_edges = set()
    lines_visited_order = []  # List to store the lines in order visited

    def dfs(vertex, visited_in_path):
        nonlocal visited_edges, lines_visited_order
        print(f"Visiting vertex {vertex}")
        visited_in_path.add(vertex)

        try:
            # Check if start and end points are in graphPoints
            if 0 <= vertex < len(points):
                start_point = points[vertex]
                for neighbor_index in graph[vertex]:
                    print(graph[vertex])
                    # Check if the edge has been visited in either direction
                    edge_key = frozenset({vertex, neighbor_index})
                    if edge_key not in visited_edges and frozenset({neighbor_index, vertex}) not in visited_edges:
                        end_point = points[neighbor_index]
                        print(points[neighbor_index])
                        # Create a line between start and end points
                        line = rs.AddLine(start_point, end_point)

                        if line:
                            # Mark the line as visited and add text dot
                            mark_line_as_visited(line, len(lines_visited_order) + 1)
                            lines_visited_order.append(line)  # Add the line to the list
                            visited_edges.add(edge_key)

                            # Recursively visit the neighbor
                            dfs(neighbor_index, visited_in_path)

        except KeyError as e:
            print(f"KeyError: {e}. Continuing...")
        
        visited_in_path.remove(vertex)
        print(f"Finished visiting vertex {vertex}")

    # Perform DFS for each unvisited point
    for start_index in range(len(points)):
        if start_index not in visited_edges:
            try:
                print(f"Starting DFS from vertex {start_index}")
                dfs(start_index, set())
                print(f"Finished DFS from vertex {start_index}")
            except KeyError as e:
                print(f"KeyError: {e}. Continuing...")

    return lines_visited_order

if __name__ == "__main__":

    sc.doc = Rhino.RhinoDoc.ActiveDoc

    visited_layer_name = "visited"
    if not rs.IsLayer(visited_layer_name):
        rs.AddLayer(visited_layer_name)

    weights_dict = add_weight_to_lines(graph)
    weights = []    
    lines = []
    for line, weight in weights_dict.items():
        weights.append(weight)
        lines.append(line)


    sorted_by_weights = dict(sorted(weights_dict.items(), key=itemgetter(1)))

    l_and_w = lines, weights

    #visited_lines = dfs_all_lines()
    sc.doc = ghdoc

    #visited_lines = visited_lines

        

    #print(nodes)






import rhinoscriptsyntax as rs
import Rhino.Geometry as rg
import scriptcontext as sc
import Rhino


def get_crv_vector(crv):
    #get the vector direction of a crv    # Create a Rhino.Geometry.Vector3d object from the start and end points
    start_point = crv.From
    end_point = crv.To
    line_vector = rg.Vector3d(end_point - start_point)
    return line_vector

def chain_linking_pt(line_a, line_b):
    #this function will determine if the point is the connceting point between two adjacent curves
    line_a_start_pt = line_a.From
    line_a_end_pt = line_a.To

    line_b_start_pt = line_b.From
    line_b_end_pt = line_b.To
    boolean = False

    if round(line_a_end_pt.X, 2) == round(line_b_start_pt.X, 2) and round(line_a_end_pt.Y, 2) == round(line_b_start_pt.Y, 2) and round(line_a_end_pt.Z, 2) == round(line_b_start_pt.Z, 2):
        boolean = True
    else:
        boolean = False

    return boolean
    
def horizontal_pt_int_test(pt, crvs):
    #test the point to see if it intersects with a point that is attached to a horizontal line in a set of curves  
    count = 0
    for crv in crvs:
        #dot = visualize_line(crv, "horizontal_pt_int_test")
        start_pt = crv.From
        end_pt = crv.To
        #testing if the start point of the curve is the same as pt
        if round(pt.X, 2) == round(start_pt.X, 2) and round(pt.Y, 2) == round(start_pt.Y, 2) and round(pt.Z, 2) == round(start_pt.Z, 2):
            #if the points are equal, test if the curve that the point is attached to is horizontal
            if start_pt.Z == end_pt.Z:
                count += 1

        #testing if the end point of the curve is the same as pt
        elif round(pt.X, 2) == round(end_pt.X, 2) and round(pt.Y, 2) == round(end_pt.Y, 2) and round(pt.Z, 2) == round(end_pt.Z, 2):
            #if the points are equal, test if the curve that the point is attached to is horizontal
            if start_pt.Z == end_pt.Z:
                count += 1

    if count >= 1:
        return False
    else:
        return True

def visualize_line(line, name):
    midpoint = line.PointAtLength(line.Length/2)
    dot = rs.AddTextDot(name, midpoint)
    return dot

def vectors_equal(line_a, line_b):
    line_a_vector = get_crv_vector(line_a)
    line_b_vector = get_crv_vector(line_b)
    line_a_vector = rs.VectorUnitize(line_a_vector)
    line_b_vector = rs.VectorUnitize(line_b_vector)

    if round(line_a_vector[0], 2) == round(line_b_vector[0], 2) and round(line_a_vector[1], 2) == round(line_b_vector[1], 2) and round(line_a_vector[2], 2) == round(line_b_vector[2], 2):
        return True
    else:
        return False  

sc.doc = Rhino.RhinoDoc.ActiveDoc

new_lines = []
new_weights = []

for i in range(len(crvs)):
    #define all line A parameters here
    line_a = crvs[i]
    #dot_a = visualize_line(line_a, "line_a")
    
    line_a_start_pt = line_a.From
    line_a_end_pt = line_a.To
    
    if line_a_start_pt.Z > line_a_end_pt.Z:
        line_a.Flip()

    for j in range(len(crvs)):
        line_b = crvs[j]
        weight = weights[i]
        alt_weight = weights[j]
        #define all line B parameters here
        added_line = False
        #dot_b = visualize_line(line_b, "line_b")
        
        line_b_start_pt = line_b.From
        line_b_end_pt = line_b.To
        line_b_midpoint = line_b.PointAtLength(line_b.Length/2)
        if line_b_start_pt.Z > line_b_end_pt.Z:
            line_b.Flip()
        #function to test if the curve pt links the two curves together
        link_pt = chain_linking_pt(line_a, line_b)
        #function to test if the vectors are equal
        are_vectors_equal = vectors_equal(line_a, line_b)
        #develop logic that defines the joining of the curves on the same vector and add weight b to the line ab
        if are_vectors_equal and link_pt:
            dist = abs(rs.Distance(line_a_start_pt, line_b_end_pt))
            line_a_horizontal_pt_int_test = horizontal_pt_int_test(line_a_end_pt, crvs)
            line_b_horizontal_pt_int_test = horizontal_pt_int_test(line_b_start_pt, crvs)
            if line_a_horizontal_pt_int_test and line_b_horizontal_pt_int_test and dist < 40:  
                    #determine if the currnet line is vertical or angled to add weight
                    #vertical = weight
                    #angled = alt_weight
                    if round(line_b_start_pt.X, 2) == round(line_b_end_pt.X, 2) and round(line_b_start_pt.Y, 2) == round(line_b_end_pt.Y, 2):
                        new_line = rg.Line(line_a_start_pt, line_b_end_pt)
                        new_lines.append(new_line)
                        new_weights.append(alt_weight + .13)
                        added_line = True
                        break
                    else:
                        new_line = rg.Line(line_a_start_pt, line_b_end_pt)
                        new_lines.append(new_line)
                        new_weights.append(alt_weight)
                        added_line = True
                        break
        #rs.DeleteObject(dot_b)
    #rs.DeleteObject(dot_a)
    if added_line is False:
        new_lines.append(line_a)
        new_weights.append(weight)



#now delete the remaining curve that overlaps the new curv 
for line_a in new_lines:
    line_a_start_pt = line_a.From
    line_a_end_pt = line_a.To
    line_a_midpoint = line_a.PointAtLength(line_a.Length/2)
    #dot_a = visualize_line(line_a, "line_a")

    if line_a.Length > 20:
        for line_b in new_lines:
            line_b_start_pt = line_b.From
            line_b_end_pt = line_b.To
            line_b_midpoint = line_b.PointAtLength(line_b.Length/2)
            #dot_b = visualize_line(line_b, "line_b")

            if line_b_start_pt.Z == line_b_end_pt.Z:
                #print("Horizontal Line")
                pass

            elif round(line_b_start_pt.X, 2) == round(line_b_end_pt.X, 2) and round(line_b_start_pt.Y, 2) == round(line_b_end_pt.Y, 2):
                pass

            else:
                link_pt = chain_linking_pt(line_a, line_b)
                are_vectors_equal = vectors_equal(line_a, line_b)
                if are_vectors_equal:
                    # test for a mid point connection and a start or end point connection
                    if round(line_a_midpoint.X, 2) == round(line_b_start_pt.X, 2) and round(line_a_midpoint.Y, 2) == round(line_b_start_pt.Y, 2) and round(line_a_midpoint.Z, 2) == round(line_b_start_pt.Z, 2):
                        if round(line_a_end_pt.X, 2) == round(line_b_end_pt.X, 2) and round(line_a_end_pt.Y, 2) == round(line_b_end_pt.Y, 2) and round(line_a_end_pt.Z, 2) == round(line_b_end_pt.Z, 2):
                            if line_b.Length < 20:
                                if isinstance(line_b, rg.Line):
                                    index = new_lines.index(line_b)
                                    new_lines.remove(line_b)
                                    new_weights.pop(index) 

                        elif round(line_a_start_pt.X, 2) == round(line_b_start_pt.X, 2) and round(line_a_end_pt.Y, 2) == round(line_b_end_pt.Y, 2) and round(line_a_end_pt.Z, 2) == round(line_b_end_pt.Z, 2):
                            if line_b.Length < 20:
                                if isinstance(line_b, rg.Line):
                                    index = new_lines.index(line_b)
                                    new_lines.remove(line_b)
                                    new_weights.pop(index) 
             
                    elif round(line_a_midpoint.X, 2) == round(line_b_end_pt.X, 2) and round(line_a_midpoint.Y, 2) == round(line_b_end_pt.Y, 2) and round(line_a_midpoint.Z, 2) == round(line_b_end_pt.Z, 2):
                        if round(line_a_end_pt.X, 2) == round(line_b_end_pt.X, 2) and round(line_a_end_pt.Y, 2) == round(line_b_end_pt.Y, 2) and round(line_a_end_pt.Z, 2) == round(line_b_end_pt.Z, 2):
                            if line_b.Length < 20:
                                if isinstance(line_b, rg.Line):
                                    index = new_lines.index(line_b)
                                    new_lines.remove(line_b)
                                    new_weights.pop(index) 

                        elif round(line_a_start_pt.X, 2) == round(line_b_start_pt.X, 2) and round(line_a_end_pt.Y, 2) == round(line_b_end_pt.Y, 2) and round(line_a_end_pt.Z, 2) == round(line_b_end_pt.Z, 2):
                            if line_b.Length < 20:
                                if isinstance(line_b, rg.Line):
                                    index = new_lines.index(line_b)
                                    new_lines.remove(line_b)
                                    new_weights.pop(index) 

                        
                    elif round(line_b_midpoint.X, 2) == round(line_a_start_pt.X, 2) and round(line_b_midpoint.Y, 2) == round(line_a_start_pt.Y, 2) and round(line_b_midpoint.Z, 2) == round(line_a_start_pt.Z, 2):
                        if round(line_a_end_pt.X, 2) == round(line_b_end_pt.X, 2) and round(line_a_end_pt.Y, 2) == round(line_b_end_pt.Y, 2) and round(line_a_end_pt.Z, 2) == round(line_b_end_pt.Z, 2):
                            if line_b.Length < 20:
                                if isinstance(line_b, rg.Line):
                                    index = new_lines.index(line_b)
                                    new_lines.remove(line_b)
                                    new_weights.pop(index) 

                        elif round(line_a_start_pt.X, 2) == round(line_b_start_pt.X, 2) and round(line_a_end_pt.Y, 2) == round(line_b_end_pt.Y, 2) and round(line_a_end_pt.Z, 2) == round(line_b_end_pt.Z, 2):
                            if line_b.Length < 20:
                                if isinstance(line_b, rg.Line):
                                    index = new_lines.index(line_b)
                                    new_lines.remove(line_b)
                                    new_weights.pop(index) 
                 
                    elif round(line_b_midpoint.X, 2) == round(line_a_end_pt.X, 2) and round(line_b_midpoint.Y, 2) == round(line_a_end_pt.Y, 2) and round(line_b_midpoint.Z, 2) == round(line_a_end_pt.Z, 2):
                        if round(line_a_end_pt.X, 2) == round(line_b_end_pt.X, 2) and round(line_a_end_pt.Y, 2) == round(line_b_end_pt.Y, 2) and round(line_a_end_pt.Z, 2) == round(line_b_end_pt.Z, 2):
                            if line_b.Length < 20:
                                if isinstance(line_b, rg.Line):
                                    index = new_lines.index(line_b)
                                    new_lines.remove(line_b)
                                    new_weights.pop(index) 

                        elif round(line_a_start_pt.X, 2) == round(line_b_start_pt.X, 2) and round(line_a_end_pt.Y, 2) == round(line_b_end_pt.Y, 2) and round(line_a_end_pt.Z, 2) == round(line_b_end_pt.Z, 2):
                            if line_b.Length < 20:
                                if isinstance(line_b, rg.Line):
                                    index = new_lines.index(line_b)
                                    new_lines.remove(line_b)
                                    new_weights.pop(index) 
            #rs.DeleteObject(dot_b)
        #rs.DeleteObject(dot_a)

sc.doc = ghdoc