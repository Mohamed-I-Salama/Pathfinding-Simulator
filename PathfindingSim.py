from tkinter import *
import heapq
import time

root = Tk()
root.geometry("750x500+300+100")  # Adjust the window size to be smaller
root.title("Pathfinding - Dijkstra's, A*, and Greedy Best-First Search algorithms")

# Variables
grid = []
matrix = []
count = 0
start, end = None, None
btn = None
rst = None
find = False
d = []
p = []
visited = []
u = -1
s = 0
e = 0
array_path = []
algorithm = StringVar(value="Dijkstra")

# Initially path array contains -1
def initialize_lists():
    global d, p, visited
    d = [float('inf')] * 570
    p = [-1] * 570
    visited = [0] * 570

initialize_lists()

# Dijkstra's algorithm
def dijkstra():
    global u, s, e
    s = start.grid_info()["row"] * 30 + start.grid_info()["column"]
    e = end.grid_info()["row"] * 30 + end.grid_info()["column"]
    d[s] = 0
    pq = [(0, s)]
    
    while pq:
        current_distance, current_node = heapq.heappop(pq)
        
        if visited[current_node]:
            continue
        visited[current_node] = 1
        show_visited(current_node)
        
        for neighbor in range(570):
            if matrix[current_node][neighbor] != float('inf'):
                distance = current_distance + matrix[current_node][neighbor]
                
                if distance < d[neighbor]:
                    d[neighbor] = distance
                    p[neighbor] = current_node
                    heapq.heappush(pq, (distance, neighbor))
        
        if current_node == e:
            break

# A* algorithm
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def astar():
    global s, e
    start_pos = (start.grid_info()["row"], start.grid_info()["column"])
    end_pos = (end.grid_info()["row"], end.grid_info()["column"])
    s = start_pos[0] * 30 + start_pos[1]
    e = end_pos[0] * 30 + end_pos[1]

    open_set = []
    heapq.heappush(open_set, (0, s))
    came_from = {}
    g_score = {i: float('inf') for i in range(570)}
    g_score[s] = 0
    f_score = {i: float('inf') for i in range(570)}
    f_score[s] = heuristic(start_pos, end_pos)
    closed_set = set()

    while open_set:
        current = heapq.heappop(open_set)[1]

        if current == e:
            reconstruct_path(came_from, current)
            return

        closed_set.add(current)
        show_visited(current)
        
        for neighbor in range(570):
            if matrix[current][neighbor] == float('inf') or neighbor in closed_set:
                continue
            tentative_g_score = g_score[current] + matrix[current][neighbor]
            if tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = g_score[neighbor] + heuristic(counter(neighbor), end_pos)
                if all(item[1] != neighbor for item in open_set):
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

# Greedy Best-First Search algorithm
def greedy_best_first_search():
    global s, e
    start_pos = (start.grid_info()["row"], start.grid_info()["column"])
    end_pos = (end.grid_info()["row"], end.grid_info()["column"])
    s = start_pos[0] * 30 + start_pos[1]
    e = end_pos[0] * 30 + end_pos[1]

    open_set = []
    heapq.heappush(open_set, (0, s))
    came_from = {}
    f_score = {i: float('inf') for i in range(570)}
    f_score[s] = heuristic(start_pos, end_pos)

    while open_set:
        current = heapq.heappop(open_set)[1]

        if current == e:
            reconstruct_path(came_from, current)
            return

        visited[current] = 1
        show_visited(current)
        for neighbor in range(570):
            if matrix[current][neighbor] == float('inf') or visited[neighbor] == 1:
                continue
            if heuristic(counter(neighbor), end_pos) < f_score[neighbor]:
                came_from[neighbor] = current
                f_score[neighbor] = heuristic(counter(neighbor), end_pos)
                if neighbor not in [i[1] for i in open_set]:
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

def reconstruct_path(came_from, current):
    global array_path
    while current in came_from:
        array_path.append(current)
        current = came_from[current]
    array_path.reverse()

# Finding path according to selected algorithm
def path(v, source):
    global array_path
    if p[v] != -1:
        path(p[v], source)
    if v != source:
        array_path.append(v)

# To store the path in the array so that we can backtrack
def display(source, n):
    for i in range(n):
        if d[i] < float('inf'):
            if i != source:
                path(i, source)
            if i != source:
                if array_path[-1] != e:
                    array_path.clear()
                else:
                    return

# This function takes a number and gives the row and column of that grid element
def counter(n):
    return divmod(n, 30)

# This will create the cost matrix
def make_matrix():
    global matrix
    matrix = []
    for i in range(570):
        matrix.append([])
        for j in range(570):
            i_r, i_c = counter(i)
            j_r, j_c = counter(j)
            if i == j:
                matrix[i].append(0)
            elif i_r == j_r:
                if (i_c == j_c - 1) or (i_c == j_c + 1):
                    if grid[i_r][i_c]["bg"] == "black" or grid[j_r][j_c]["bg"] == "black":
                        matrix[i].append(float('inf'))
                    else:
                        matrix[i].append(1)
                else:
                    matrix[i].append(float('inf'))
            elif i_c == j_c:
                if (i_r == j_r - 1) or (i_r == j_r + 1):
                    if grid[i_r][i_c]["bg"] == "black" or grid[j_r][j_c]["bg"] == "black":
                        matrix[i].append(float('inf'))
                    else:
                        matrix[i].append(1)
                else:
                    matrix[i].append(float('inf'))
            else:
                matrix[i].append(float('inf'))

# Creating the grid of buttons
def make_grid():
    global grid
    grid = []
    for i in range(19):
        row = []
        for j in range(30):
            b = Button(root, bg="red", width=2, height=1, bd=1)  # Adjust height to better fit the space
            b.grid(row=i, column=j, padx=0, pady=0)  # Remove padding to eliminate white bars
            b.bind("<Button-1>", click)
            row.append(b)
        grid.append(row)

# Restart Functionality
def restart():
    global start, end, count, find, u, s, e, array_path
    start = None
    end = None
    for row in grid:
        for button in row:
            button["bg"] = "red"
            button["state"] = "normal"  # Reset button state to normal
    count = 0
    find = False
    u = -1
    s = 0
    e = 0
    array_path.clear()
    initialize_lists()

# For backtracking the path
def backtrack():
    if len(array_path) == 0:
        print("No Path Found")
    global find
    for ele in array_path[:-1]:
        r, col = counter(ele)
        grid[r][col]["bg"] = "light blue"
    find = True

# Start the simulation
def start_func():
    start_time = time.time()
    if count == 0 or count == 1:
        return
    if not find:
        make_matrix()
        if algorithm.get() == "Dijkstra":
            dijkstra()
            display(s, 570)
        elif algorithm.get() == "A*":
            astar()
        else:
            greedy_best_first_search()
        backtrack()
    end_time = time.time()
    runtime_label.config(text=f"Runtime: {end_time - start_time:.1f}s")

# On click functionality for button
def click(event):
    global count, start, end, grid
    button = event.widget
    row, col = button.grid_info()["row"], button.grid_info()["column"]
    if button["bg"] == "red" and not (start and end):
        if count == 0:
            button["bg"] = "green"
            start = button
        elif count == 1:
            button["bg"] = "blue"
            end = button
        count += 1
    elif button["bg"] == "red":
        button["bg"] = "black"
    elif button["bg"] == "black":
        button["bg"] = "red"

# from tkinter import *
import heapq
import time

root = Tk()
root.geometry("750x500+300+100")  # Adjust the window size to be smaller
root.title("Pathfinding - Dijkstra's, A*, and Greedy Best-First Search algorithms")

# Variables
grid = []
matrix = []
count = 0
start, end = None, None
btn = None
rst = None
find = False
d = []
p = []
visited = []
u = -1
s = 0
e = 0
array_path = []
algorithm = StringVar(value="Dijkstra")

# Initially path array contains -1
def initialize_lists():
    global d, p, visited
    d = [float('inf')] * 570
    p = [-1] * 570
    visited = [0] * 570

initialize_lists()

# Dijkstra's algorithm
def dijkstra():
    global u, s, e
    s = start.grid_info()["row"] * 30 + start.grid_info()["column"]
    e = end.grid_info()["row"] * 30 + end.grid_info()["column"]
    d[s] = 0
    pq = [(0, s)]
    
    while pq:
        current_distance, current_node = heapq.heappop(pq)
        
        if visited[current_node]:
            continue
        visited[current_node] = 1
        show_visited(current_node)
        
        for neighbor in range(570):
            if matrix[current_node][neighbor] != float('inf'):
                distance = current_distance + matrix[current_node][neighbor]
                
                if distance < d[neighbor]:
                    d[neighbor] = distance
                    p[neighbor] = current_node
                    heapq.heappush(pq, (distance, neighbor))
        
        if current_node == e:
            break

# A* algorithm
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def astar():
    global s, e
    start_pos = (start.grid_info()["row"], start.grid_info()["column"])
    end_pos = (end.grid_info()["row"], end.grid_info()["column"])
    s = start_pos[0] * 30 + start_pos[1]
    e = end_pos[0] * 30 + end_pos[1]

    open_set = []
    heapq.heappush(open_set, (0, s))
    came_from = {}
    g_score = {i: float('inf') for i in range(570)}
    g_score[s] = 0
    f_score = {i: float('inf') for i in range(570)}
    f_score[s] = heuristic(start_pos, end_pos)
    closed_set = set()

    while open_set:
        current = heapq.heappop(open_set)[1]

        if current == e:
            reconstruct_path(came_from, current)
            return

        closed_set.add(current)
        show_visited(current)
        
        for neighbor in range(570):
            if matrix[current][neighbor] == float('inf') or neighbor in closed_set:
                continue
            tentative_g_score = g_score[current] + matrix[current][neighbor]
            if tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = g_score[neighbor] + heuristic(counter(neighbor), end_pos)
                if all(item[1] != neighbor for item in open_set):
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

# Greedy Best-First Search algorithm
def greedy_best_first_search():
    global s, e
    start_pos = (start.grid_info()["row"], start.grid_info()["column"])
    end_pos = (end.grid_info()["row"], end.grid_info()["column"])
    s = start_pos[0] * 30 + start_pos[1]
    e = end_pos[0] * 30 + end_pos[1]

    open_set = []
    heapq.heappush(open_set, (0, s))
    came_from = {}
    f_score = {i: float('inf') for i in range(570)}
    f_score[s] = heuristic(start_pos, end_pos)

    while open_set:
        current = heapq.heappop(open_set)[1]

        if current == e:
            reconstruct_path(came_from, current)
            return

        visited[current] = 1
        show_visited(current)
        for neighbor in range(570):
            if matrix[current][neighbor] == float('inf') or visited[neighbor] == 1:
                continue
            if heuristic(counter(neighbor), end_pos) < f_score[neighbor]:
                came_from[neighbor] = current
                f_score[neighbor] = heuristic(counter(neighbor), end_pos)
                if neighbor not in [i[1] for i in open_set]:
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

def reconstruct_path(came_from, current):
    global array_path
    while current in came_from:
        array_path.append(current)
        current = came_from[current]
    array_path.reverse()

# Finding path according to selected algorithm
def path(v, source):
    global array_path
    if p[v] != -1:
        path(p[v], source)
    if v != source:
        array_path.append(v)

# To store the path in the array so that we can backtrack
def display(source, n):
    for i in range(n):
        if d[i] < float('inf'):
            if i != source:
                path(i, source)
            if i != source:
                if array_path[-1] != e:
                    array_path.clear()
                else:
                    return

# This function takes a number and gives the row and column of that grid element
def counter(n):
    return divmod(n, 30)

# This will create the cost matrix
def make_matrix():
    global matrix
    matrix = []
    for i in range(570):
        matrix.append([])
        for j in range(570):
            i_r, i_c = counter(i)
            j_r, j_c = counter(j)
            if i == j:
                matrix[i].append(0)
            elif i_r == j_r:
                if (i_c == j_c - 1) or (i_c == j_c + 1):
                    if grid[i_r][i_c]["bg"] == "black" or grid[j_r][j_c]["bg"] == "black":
                        matrix[i].append(float('inf'))
                    else:
                        matrix[i].append(1)
                else:
                    matrix[i].append(float('inf'))
            elif i_c == j_c:
                if (i_r == j_r - 1) or (i_r == j_r + 1):
                    if grid[i_r][i_c]["bg"] == "black" or grid[j_r][j_c]["bg"] == "black":
                        matrix[i].append(float('inf'))
                    else:
                        matrix[i].append(1)
                else:
                    matrix[i].append(float('inf'))
            else:
                matrix[i].append(float('inf'))

# Creating the grid of buttons
def make_grid():
    global grid
    grid = []
    for i in range(19):
        row = []
        for j in range(30):
            b = Button(root, bg="red", width=2, height=1, bd=1)  # Adjust height to better fit the space
            b.grid(row=i, column=j, padx=0, pady=0)  # Remove padding to eliminate white bars
            b.bind("<Button-1>", click)
            row.append(b)
        grid.append(row)

# This is Restart Functionality
def restart():
    global start, end, count, find, u, s, e, array_path
    start = None
    end = None
    for row in grid:
        for button in row:
            button["bg"] = "red"
            button["state"] = "normal"  # Reset button state to normal
    count = 0
    find = False
    u = -1
    s = 0
    e = 0
    array_path.clear()
    initialize_lists()

# For backtracking the path
def backtrack():
    if len(array_path) == 0:
        print("No Path Found")
    global find
    for ele in array_path[:-1]:
        r, col = counter(ele)
        grid[r][col]["bg"] = "light blue"
    find = True

# Start the simulation
def start_func():
    start_time = time.time()
    if count == 0 or count == 1:
        return
    if not find:
        make_matrix()
        if algorithm.get() == "Dijkstra":
            dijkstra()
            display(s, 570)
        elif algorithm.get() == "A*":
            astar()
        else:
            greedy_best_first_search()
        backtrack()
    end_time = time.time()
    runtime_label.config(text=f"Runtime: {end_time - start_time:.1f}s")

# On click functionality for button
def click(event):
    global count, start, end, grid
    button = event.widget
    row, col = button.grid_info()["row"], button.grid_info()["column"]
    if button["bg"] == "red" and not (start and end):
        if count == 0:
            button["bg"] = "green"
            start = button
        elif count == 1:
            button["bg"] = "blue"
            end = button
        count += 1
    elif button["bg"] == "red":
        button["bg"] = "black"
    elif button["bg"] == "black":
        button["bg"] = "red"

# Showing visited nodes in yellow
def show_visited(current_node):
    row, col = counter(current_node)
    if grid[row][col]["bg"] not in ("green", "blue"):
        grid[row][col]["bg"] = "yellow"
    root.update()
    time.sleep(0.01)

# Algorithm dropdown menu
algorithm_menu = OptionMenu(root, algorithm, "Dijkstra", "A*", "Greedy Best-First Search")
algorithm_menu.grid(row=19, column=0, columnspan=10, sticky="ew")

# Restart button
restart_button = Button(root, text="RESTART", command=restart)
restart_button.grid(row=19, column=10, columnspan=10, sticky="ew")

# Start button
start_button = Button(root, text="START", command=start_func)
start_button.grid(row=19, column=20, columnspan=10, sticky="ew")

# Runtime display
runtime_label = Label(root, text="Runtime: 0.0s")
runtime_label.grid(row=19, column=30, columnspan=10, sticky="ew")

make_grid()
root.mainloop()isited nodes in yellow
def show_visited(current_node):
    row, col = counter(current_node)
    if grid[row][col]["bg"] not in ("green", "blue"):
        grid[row][col]["bg"] = "yellow"
    root.update()
    time.sleep(0.01)

# Algorithm dropdown menu
algorithm_menu = OptionMenu(root, algorithm, "Dijkstra", "A*", "Greedy Best-First Search")
algorithm_menu.grid(row=19, column=0, columnspan=10, sticky="ew")

# Restart button
restart_button = Button(root, text="RESTART", command=restart)
restart_button.grid(row=19, column=10, columnspan=10, sticky="ew")

# Start button
start_button = Button(root, text="START", command=start_func)
start_button.grid(row=19, column=20, columnspan=10, sticky="ew")

# Runtime display
runtime_label = Label(root, text="Runtime: 0.0s")
runtime_label.grid(row=19, column=30, columnspan=10, sticky="ew")

make_grid()
root.mainloop()
