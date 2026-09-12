# path_planner.py — A* with unknown-space penalty and route metrics
import heapq, math
from config import GRID_SIZE, OCCUPIED, UNKNOWN, CELL_CM

def astar(grid, start, goal, unknown_cost=1.5):
    if not (0 <= start[0] < GRID_SIZE and 0 <= start[1] < GRID_SIZE): return None
    if not (0 <= goal[0] < GRID_SIZE and 0 <= goal[1] < GRID_SIZE): return None
    def h(a,b): return abs(a[0]-b[0])+abs(a[1]-b[1])
    open_set=[(h(start,goal),0,start)]; came={}; g={start:0}; closed=set()
    while open_set:
        _,cur_g,current=heapq.heappop(open_set)
        if current in closed: continue
        closed.add(current)
        if current==goal:
            path=[current]
            while current in came: current=came[current]; path.append(current)
            return path[::-1]
        for dx,dy in ((1,0),(-1,0),(0,1),(0,-1)):
            nxt=(current[0]+dx,current[1]+dy)
            if not (0<=nxt[0]<GRID_SIZE and 0<=nxt[1]<GRID_SIZE): continue
            if grid[nxt[0]][nxt[1]] >= OCCUPIED: continue
            cost=1.0 + (unknown_cost-1.0 if grid[nxt[0]][nxt[1]] == UNKNOWN else 0)
            ng=cur_g+cost
            if ng<g.get(nxt,float('inf')):
                g[nxt]=ng; came[nxt]=current; heapq.heappush(open_set,(ng+h(nxt,goal),ng,nxt))
    return None

def path_to_instructions(path,start_heading,step_cm=65.0):
    if not path or len(path)<2: return []
    direction={(1,0):90,(-1,0):270,(0,-1):0,(0,1):180}
    out=[]; heading=float(start_heading); cells=0
    def flush():
        nonlocal cells
        if cells:
            out.append(('walk', max(1, int(math.ceil(cells * CELL_CM / max(float(step_cm), 1.0))))))
            cells=0
    for a,b in zip(path,path[1:]):
        needed=direction[(b[0]-a[0],b[1]-a[1])]
        turn=(needed-heading+360)%360
        if turn>180: turn-=360
        if abs(turn)>1:
            flush(); out.append(('turn',turn)); heading=needed
        cells+=1
    flush()
    return out

def instruction_to_speech(instructions):
    """Turn ('walk', n) / ('turn', degrees) instructions into TTS sentences,
    e.g. 'Walk 6 steps.' 'Turn right.' ... 'Destination ahead.'"""
    sentences = []
    for kind, value in instructions:
        if kind == 'walk':
            sentences.append(f'Walk {value} step{"s" if value != 1 else ""}.')
        elif kind == 'turn':
            sentences.append('Turn right.' if value > 0 else 'Turn left.')
    if sentences:
        sentences.append('Destination ahead.')
    return sentences


def route_metrics(instructions):
    walk=sum(v for k,v in instructions if k=='walk')
    turns=sum(1 for k,_ in instructions if k=='turn')
    complexity=round(turns*0.5 + walk/20.0,2)
    return walk,turns,complexity
