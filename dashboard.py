# dashboard.py — local-only Flask dashboard
from flask import Flask, jsonify, render_template_string
from database import get_recent_sessions, get_recent_confidence
from progression import level_for_confidence
app=Flask(__name__)
HTML='''<!doctype html><html><head><meta name="viewport" content="width=device-width,initial-scale=1"><title>EchoMap Dashboard</title></head><body><h1>EchoMap Progress</h1><p>Confidence: <b>{{conf}}</b> &nbsp; Level: <b>{{level}}</b></p><table border="1" cellpadding="6"><tr><th>Destination</th><th>Success</th><th>Errors</th><th>Reaction</th><th>Complexity</th><th>Confidence</th></tr>{% for r in rows %}<tr><td>{{r[1]}}</td><td>{{r[4]}}</td><td>{{r[3]}}</td><td>{{'%.2f'%r[6]}}</td><td>{{'%.2f'%r[8]}}</td><td>{{'%.2f'%r[5]}}</td></tr>{% endfor %}</table></body></html>'''
@app.route('/')
def index():
 c=get_recent_confidence(); return render_template_string(HTML,conf=c,level=level_for_confidence(c),rows=get_recent_sessions(20))
@app.route('/api/sessions')
def sessions(): return jsonify(get_recent_sessions(50))

def run_dashboard(host='0.0.0.0',port=5000): app.run(host=host,port=port,debug=False,use_reloader=False)
