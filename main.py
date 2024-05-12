import pygame
import heapq
from collections import deque
import random
import argparse


pygame.init()

WHITE = (255, 255, 255)
GRAY = (142, 145, 143)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
YELLOW = (255, 255, 0)

size = [800, 800]
screen = pygame.display.set_mode(size)

done = False
clock = pygame.time.Clock()

# 기본값
N = 30
M = 30
obstacle_ratio = 0.2

# 커맨드로 입력받기
parser = argparse.ArgumentParser(description='A* Pathfinding Visualizer')
parser.add_argument('--N', type=int, default=30, help='Number of rows in the grid (default: 30)')
parser.add_argument('--M', type=int, default=30, help='Number of columns in the grid (default: 30)')
parser.add_argument('--O', type=float, default=0.2, help='Ratio of obstacles in the grid (default: 0.2)')
args = parser.parse_args()

# 커맨드로 입력받고 넣는 과정
N = args.N
M = args.M
obstacle_ratio = args.O


# 화면크기
screen_width = 800
screen_height = 800

node_area_width = 600
node_area_height = 600

# 가로세로선 개수
garo_num_of_lines = N     #행 개수
sero_num_of_lines = M     #열 개수

# 노드존 꽉채우기
garo_gap = node_area_width // garo_num_of_lines
sero_gap = node_area_width // sero_num_of_lines

#0으로 초기화
matrix = [[0 for j in range(garo_num_of_lines)] for i in range(sero_num_of_lines)]

start_point = (0, 0)
end_point = (1, 1)

# 시작점과 종료점의 매트릭스 값 설정
matrix[start_point[1]][start_point[0]] = 'S'
matrix[end_point[1]][end_point[0]] = 'G'

dragging_start = False
dragging_end = False

# 버튼을 위한 폰트 설정
button_font = pygame.font.Font(None, 30)

# 버튼 크기 설정
button_width = 150
button_height = 50

# A*버튼 생성 및 위치 설정
start_button_rect = pygame.Rect(40, 700, button_width, button_height)
start_button_text = "Start A* Search"
start_button = (start_button_rect, start_button_text)

# Random_walls_button 생성
random_walls_button_rect = pygame.Rect(220, 700, button_width, button_height)
random_walls_button_text = "Random walls"
random_walls_button = (random_walls_button_rect, random_walls_button_text)

#reset 버튼
reset_button_rect = pygame.Rect(400, 700, button_width, button_height)
reset_button_text = "Reset"
reset_button = (reset_button_rect, reset_button_text)

# 맨해튼과 유클리드
manhattan_button_rect = pygame.Rect(650, 100, 100, 50)
euclidean_button_rect = pygame.Rect(650, 200, 100, 50)

manhattan_button_text = "Manhattan"
euclidean_button_text = "Euclidean"

def print_distance(distance):
# 폰트 만들기

    font = pygame.font.Font(None, 36)  # None은 기본 시스템 폰트를 사용하겠다는 의미입니다.
    text = font.render(str(distance), True, BLACK)  # 정수형 변수를 문자열로 변환하고, 텍스트를 렌더링합니다.
    # 텍스트의 렌더링된 사각 영역 가져오기
    text_rect = text.get_rect()
    # 텍스트를 화면의 우측 중앙에 위치하도록 설정 
    text_rect.midright = (screen.get_width() - 20, screen.get_height() // 2)  
    # 텍스트를 화면에 그리기
    pygame.draw.rect(screen, WHITE, (800 // 2, 0, 800 // 2, 800))
    screen.blit(text, text_rect)  # 텍스트를 화면에 그립니다.
    
    
def astarTamseck(matrix, man_uclid):
    # 이니셜라이즈드
    start_point = None
    end_point = None
    for x in range(sero_num_of_lines):
            for y in range(garo_num_of_lines):
                if matrix[x][y] == 'S':
                    start_point = (x,y)
                elif matrix[x][y] == 'G':
                    end_point = (x,y)

    open_list = []
    closed_list = set()
    heapq.heappush(open_list, (0, start_point))   #우선순위큐
    parent = {}                     #이전노드 저장
    g_score = {start_point: 0}      #시작점에서 현재위치까지
    explored_node_count = 0


    # 휴리스틱 함수 수직수평거리
    def heuristic(node):
        return abs(node[0] - end_point[0]) + abs(node[1] - end_point[1])
    
    def heuristic2(node):
        dx = abs(node[0] - end_point[0])
        dy = abs(node[1] - end_point[1])
        distance = dx ** 2 + dy ** 2
        distance = distance**0.5
        print(round(distance,2))
        return round(distance,2)    
    
    #상하좌우 가능한지 
    def get_neighbors(node):
        x, y = node
        neighbors = [(x+1, y), (x-1, y), (x, y+1), (x, y-1)]  # 상하좌우 이웃 위치
        valid_neighbors = [(nx, ny) for nx, ny in neighbors if 0 <= nx < len(matrix) and 0 <= ny < len(matrix[0]) and matrix[nx][ny] != 1]
        return valid_neighbors
    
    while open_list:
        current_cost, current_node = heapq.heappop(open_list)

        if current_node == end_point: #현 노드가 도착점인 경우
            path = []
            while current_node in parent:
                path.append(current_node)   #경로 리스트에 현재 노드를 추가
                current_node = parent[current_node]
            path.append(start_point) # 시작점을 경로 리스트에 추가
            return path[::-1], explored_node_count, True  # 역순으로 반환하여 시작점에서 도착점까지의 경로 반환
        
        closed_list.add(current_node) #현재노드 방문처리
        explored_node_count = len(closed_list)  # 방문한 노드의 개수
        for neighbor in get_neighbors(current_node): # 그노드의 기점에서 가능한 노드 탐색
            if neighbor in closed_list:
                continue                #이미 방문한 노드 무시

            tentative_g_score = g_score[current_node] + 1  # 이동 비용은 항상 1로 가정
            #이웃노드가 open리스트에 없거나 있어도 이웃노드의 이동비용이 낮을경우
            if neighbor not in [node[1] for node in open_list] or tentative_g_score < g_score.get(neighbor, float('inf')):
                g_score[neighbor] = tentative_g_score
                f_score = tentative_g_score + heuristic(neighbor) #현위치에서 도착점까지 추정거리(맨해튼)
                f_score2 = tentative_g_score + heuristic2(neighbor) #현위치에서 도착점까지 추정거리(유클리드)
                if man_uclid == 0: #맨해튼 거리출력일떄
                    print_distance(f_score)
                if man_uclid == 1: #유클리드 거리출력일때
                    print_distance(f_score2)
                heapq.heappush(open_list, (f_score, neighbor)) #추정거리를 우선순위큐에 넣기
                parent[neighbor] = current_node

    if not open_list:

        visited = set()

        queue = deque([(start_point, [])]) 

        f_values = {start_point: 0}  # 시작 노드의 f 값은 0으로 초기화
        while queue:
            current_node, path = queue.popleft()  # 큐에서 노드와 해당 노드까지의 경로를 가져옴
        
            if current_node not in visited:  # 방문하지 않은 노드일 경우
                visited.add(current_node)  # 방문한 노드로 표시
            for neighbor in get_neighbors(current_node):  # 이웃 노드를 순회하며
                if neighbor not in visited:  # 방문하지 않은 이웃 노드만 추가
                    queue.append((neighbor, path + [current_node]))  # 이웃 노드와 경로를 큐에 추가
                    # 각 노드에 대한 f 값을 업데이트
                    f_values[neighbor] = len(path) + 1 + heuristic(current_node)

        # 가장 낮은 f 값을 가진 노드와 해당 노드까지의 경로를 찾음
        min_f_node = min(f_values, key=f_values.get)
        min_f_path = path + [min_f_node] if min_f_node else None
        return min_f_path, explored_node_count, False            

def draw_path(screen, path, gap):
    for x in range(sero_num_of_lines):
        for y in range(garo_num_of_lines):
            if matrix[x][y] == 0:
                pygame.draw.rect(screen, WHITE, (y * garo_gap, x * sero_gap, garo_gap, sero_gap))

    for i in range(len(path) - 1):
        x1, y1 = path[i]
        x2, y2 = path[i + 1]
        pygame.draw.line(screen, YELLOW, (y1 * garo_gap + garo_gap // 2, x1 * sero_gap + sero_gap // 2), (y2 * garo_gap + garo_gap // 2, x2 * sero_gap + sero_gap // 2), 5)
    
# Randomly place obstacles in the matrix
def place_obstacles(matrix, obstacle_ratio):
    total_cells = sero_num_of_lines * garo_num_of_lines
    num_obstacles = int(total_cells * obstacle_ratio)

    for _ in range(num_obstacles):
        x = random.randint(0, sero_num_of_lines - 1)
        y = random.randint(0, garo_num_of_lines - 1)

        if matrix[x][y] == 0:
            matrix[x][y] = 1
    
    matrix[start_point[1]][start_point[0]] = 'S'
    matrix[end_point[1]][end_point[0]] = 'G'


    
screen.fill(WHITE)

def runGame():
    global done, dragging_start, dragging_end, path_true, path1, closed_node, man_uclid
    path_true = False
    man_uclid = 0
    while not done:
        clock.tick(10)
        # 버튼 그리기
        #A*버튼
        pygame.draw.rect(screen, GRAY, start_button_rect)
        text_surface = button_font.render(start_button_text, True, BLACK)
        text_rect = text_surface.get_rect(center=start_button_rect.center)
        screen.blit(text_surface, text_rect)

        #random wall 버튼
        pygame.draw.rect(screen, GRAY, random_walls_button_rect)
        text_surface = button_font.render(random_walls_button_text, True, BLACK)
        text_rect = text_surface.get_rect(center=random_walls_button_rect.center)
        screen.blit(text_surface, text_rect)

        #Reset 버튼
        pygame.draw.rect(screen, GRAY, reset_button_rect)
        text_surface = button_font.render(reset_button_text, True, BLACK)
        text_rect = text_surface.get_rect(center=reset_button_rect.center)
        screen.blit(text_surface, text_rect)

        #맨해튼과 유클리드
        pygame.draw.rect(screen, GRAY, manhattan_button_rect)
        pygame.draw.rect(screen, GRAY, euclidean_button_rect)

        manhattan_text_surface = button_font.render(manhattan_button_text, True, BLACK)
        manhattan_text_rect = manhattan_text_surface.get_rect(center=manhattan_button_rect.center)
        screen.blit(manhattan_text_surface, manhattan_text_rect)

        euclidean_text_surface = button_font.render(euclidean_button_text, True, BLACK)
        euclidean_text_rect = euclidean_text_surface.get_rect(center=euclidean_button_rect.center)
        screen.blit(euclidean_text_surface, euclidean_text_rect)
        


    # 마우스 이벤트
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                done = True
            if event.type == pygame.MOUSEBUTTONDOWN:
                mouse_pos = pygame.mouse.get_pos()

                #A* 버튼 클릭 처리
                if start_button_rect.collidepoint(mouse_pos):
                    print("A*star click")
                    path, closed_node, success_fail = astarTamseck(matrix, man_uclid)
                    if success_fail:
                         draw_path(screen, path, garo_gap)
                         path_true = True
                         path1 = path
                         print("탐색한 노드 갯수 : ", closed_node) 

                    else:
                         print("경로를 찾을 수 없음")
                         draw_path(screen, path, garo_gap)
                         path_true = True
                         path1 = path
                         print("탐색한 노드 갯수 : ", closed_node) 

                # "random wall"버튼 클릭처리
                if random_walls_button_rect.collidepoint(mouse_pos):
                    print("random wall click")
                    for i in range(sero_num_of_lines):
                         for j in range(garo_num_of_lines):
                            matrix[i][j] = 0
                    path1 = ()
                    path_true = False

                    total_cells = sero_num_of_lines * garo_num_of_lines
                    num_obstacles = int(total_cells * obstacle_ratio)

                    for _ in range(num_obstacles):
                        x = random.randint(0, sero_num_of_lines - 1)
                        y = random.randint(0, garo_num_of_lines - 1)

                        if matrix[x][y] == 0:
                            matrix[x][y] = 1
    
                        matrix[start_point[1]][start_point[0]] = 'S'
                        matrix[end_point[1]][end_point[0]] = 'G'
                    path1 = ()
                    path_true = False

                # "reset"버튼 클릭처리
                if reset_button_rect.collidepoint(mouse_pos):
                    print("reset button click")
                    for i in range(sero_num_of_lines):
                         for j in range(garo_num_of_lines):
                            matrix[i][j] = 0
                 # 시작점과 도착점의 위치에 각각 'S'와 'G' 값을 넣어줌
                    matrix[start_point[1]][start_point[0]] = 'S'
                    matrix[end_point[1]][end_point[0]] = 'G'
                    path1 = ()
                    path_true = False
                    

                # 맨해튼 버튼 클릭 처리
                if manhattan_button_rect.collidepoint(mouse_pos):
                    print("Manhattan button clicked")
                    man_uclid = 0  #0이면 맨해튼

                # 유클리디안 버튼 클릭 처리
                elif euclidean_button_rect.collidepoint(mouse_pos):
                    print("Euclidean button clicked")
                    man_uclid = 1  #1이면 유클리드

                mouse_x = mouse_pos[1] // sero_gap
                mouse_y = mouse_pos[0] // garo_gap
                
                if 0 <= mouse_x < sero_num_of_lines and 0 <= mouse_y < garo_num_of_lines:
                    if matrix[mouse_x][mouse_y] == 0:
                        matrix[mouse_x][mouse_y] = 1
                        path1 = ()
                    elif matrix[mouse_x][mouse_y] == 1:
                        matrix[mouse_x][mouse_y] = 0
                        path1 = ()
                    elif matrix[mouse_x][mouse_y] == 'S':
                        dragging_start = True
                    elif matrix[mouse_x][mouse_y] == 'G':
                        dragging_end = True

            if event.type == pygame.MOUSEBUTTONUP:
                dragging_start = False
                dragging_end = False
            if event.type == pygame.MOUSEMOTION:
                if dragging_start:
                    matrix[mouse_x][mouse_y] = 0
                    mouse_pos = pygame.mouse.get_pos()
                    mouse_x = mouse_pos[1] // sero_gap
                    mouse_y = mouse_pos[0] // garo_gap
                    if 0 <= mouse_x < sero_num_of_lines and 0 <= mouse_y < garo_num_of_lines:
                        matrix[mouse_x][mouse_y] = 'S'
                        path1 = ()
                        path_true = False

                elif dragging_end:
                    matrix[mouse_x][mouse_y] = 0
                    mouse_pos = pygame.mouse.get_pos()
                    mouse_x = mouse_pos[1] // sero_gap
                    mouse_y = mouse_pos[0] // garo_gap
                    if 0 <= mouse_x < sero_num_of_lines and 0 <= mouse_y < garo_num_of_lines:
                        matrix[mouse_x][mouse_y] = 'G'
                        path1 = ()
                        path_true = False

        font = pygame.font.Font(None, 20)
        for x in range(sero_num_of_lines):
            for y in range(garo_num_of_lines):
                if matrix[x][y] == 'S':
                    pygame.draw.rect(screen, GREEN, (y * garo_gap, x * sero_gap, garo_gap, sero_gap))
                    text_surface = font.render(str(matrix[x][y]), True, BLACK)
                    screen.blit(text_surface, (y * garo_gap + garo_gap // 2, x * sero_gap + sero_gap // 2))
                elif matrix[x][y] == 'G':
                    pygame.draw.rect(screen, RED, (y * garo_gap, x * sero_gap, garo_gap, sero_gap))
                    text_surface = font.render(str(matrix[x][y]), True, BLACK)
                    screen.blit(text_surface, (y * garo_gap + garo_gap // 2, x * sero_gap + sero_gap // 2))
                elif matrix[x][y] == 1:
                    pygame.draw.rect(screen, GRAY, (y * garo_gap, x * sero_gap, garo_gap, sero_gap))
                elif path_true and (x,y) in path1:
                    if matrix[x][y] == 1:
                        pygame.draw.rect(screen, GRAY, (y * garo_gap, x * sero_gap, garo_gap, sero_gap))
                    continue   
                elif matrix[x][y] == 0:
                    pygame.draw.rect(screen, WHITE, (y * garo_gap, x * sero_gap, garo_gap, sero_gap))
        #격자그리기
        for y_idx in range(garo_num_of_lines + 1):
            y_pos = y_idx * garo_gap
            pygame.draw.line(screen, BLACK, (0, y_pos), (garo_gap * garo_num_of_lines, y_pos), 3)

        for x_idx in range(sero_num_of_lines + 1):
            x_pos = x_idx * sero_gap
            pygame.draw.line(screen, BLACK, (x_pos, 0), (x_pos, sero_gap * sero_num_of_lines), 3)        
                
        pygame.display.update()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                done = True

runGame()
pygame.quit()