# database.py — local SQLite storage for landmarks, paths and session metrics
import sqlite3, json, os
from datetime import datetime
from config import DB_PATH

SESSION_COLUMNS = {
    'reaction_time': 'REAL DEFAULT 0', 'error_rate': 'REAL DEFAULT 0',
    'route_complexity': 'REAL DEFAULT 0', 'route_steps': 'INTEGER DEFAULT 0',
    'turns': 'INTEGER DEFAULT 0', 'dynamic_obstacles': 'INTEGER DEFAULT 0',
    'level': 'INTEGER DEFAULT 1', 'confidence_score': 'REAL DEFAULT 0.5'
}

def _conn():
    os.makedirs(os.path.dirname(DB_PATH), exist_ok=True)
    return sqlite3.connect(DB_PATH)

def init_db():
    conn=_conn(); c=conn.cursor()
    c.execute('''CREATE TABLE IF NOT EXISTS landmarks (
        name TEXT PRIMARY KEY, grid_x INTEGER, grid_y INTEGER, map_data TEXT, created_at TEXT)''')
    c.execute('''CREATE TABLE IF NOT EXISTS sessions (
        id INTEGER PRIMARY KEY AUTOINCREMENT, start_time TEXT, end_time TEXT,
        destination TEXT, steps_taken INTEGER, errors INTEGER, success INTEGER,
        confidence_score REAL DEFAULT 0.5)''')
    for col, spec in SESSION_COLUMNS.items():
        try: c.execute(f'ALTER TABLE sessions ADD COLUMN {col} {spec}')
        except sqlite3.OperationalError: pass
    conn.commit(); conn.close(); print('[DB] Ready.')

def save_landmark(name, grid_x, grid_y, grid_obj):
    name=name.lower().strip()
    conn=_conn(); conn.execute('INSERT OR REPLACE INTO landmarks VALUES (?,?,?,?,?)',
        (name, grid_x, grid_y, json.dumps(grid_obj.to_dict()), datetime.now().isoformat()))
    conn.commit(); conn.close(); print(f"[DB] Saved '{name}' at ({grid_x},{grid_y})")

def get_landmark(name):
    conn=_conn(); row=conn.execute('SELECT grid_x,grid_y,map_data FROM landmarks WHERE name=?',(name.lower().strip(),)).fetchone(); conn.close(); return row

def list_landmarks():
    conn=_conn(); rows=conn.execute('SELECT name,grid_x,grid_y FROM landmarks ORDER BY created_at').fetchall(); conn.close(); return rows

def delete_landmark(name):
    conn=_conn(); conn.execute('DELETE FROM landmarks WHERE name=?',(name.lower().strip(),)); conn.commit(); conn.close()

def log_session(start_time, destination, steps, errors, success, reaction_time=0, route_complexity=0,
                route_steps=0, turns=0, dynamic_obstacles=0, level=1, confidence=None):
    if confidence is None:
        # Baseline only; confidence_model.py can replace this with the recurrent prediction.
        confidence=max(0.0,min(1.0, 0.65 - 0.08*errors + 0.02*success))
    total=max(steps,1); error_rate=errors/total
    conn=_conn(); conn.execute('''INSERT INTO sessions
      (start_time,end_time,destination,steps_taken,errors,success,confidence_score,
       reaction_time,error_rate,route_complexity,route_steps,turns,dynamic_obstacles,level)
      VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?)''',
      (start_time,datetime.now().isoformat(),destination,steps,errors,int(bool(success)),confidence,
       reaction_time,error_rate,route_complexity,route_steps,turns,dynamic_obstacles,level))
    conn.commit(); conn.close(); print(f'[DB] Session logged. Confidence={confidence:.2f}'); return confidence

def get_recent_sessions(n=10):
    conn=_conn(); rows=conn.execute('''SELECT id,destination,steps_taken,errors,success,confidence_score,
      reaction_time,error_rate,route_complexity,turns,dynamic_obstacles,level,start_time,end_time
      FROM sessions ORDER BY id DESC LIMIT ?''',(n,)).fetchall(); conn.close(); return rows

def get_recent_confidence(n=5):
    rows=get_recent_sessions(n); vals=[r[5] for r in rows if r[5] is not None]
    return round(sum(vals)/len(vals),2) if vals else 0.5
