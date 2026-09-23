#!/usr/bin/env python3
"""Desktop Town05 scenario editor. Draft first, explicitly run second."""
import concurrent.futures
import fcntl
import json
import math
from pathlib import Path
import sys
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import uuid

from scenario_gui_backend import ScenarioEngine, STATE, ROOT, TRAFFIC_CONFIG, atomic_json, read_map, validate_scenario

BG = '#101923'
PANEL = '#182431'
TEXT = '#e9f0f4'
MUTED = '#98adba'
ACCENT = '#41cfbc'
MODE_LABELS = {'parked': '정지 차량', 'autopilot': '차로 따라 주행', 'wait': '제자리 대기', 'walk': '목표 지점까지 걷기'}


class ScenarioGUI:
    def __init__(self, root, engine=None):
        self.root = root
        self.engine = engine or ScenarioEngine()
        self.executor = concurrent.futures.ThreadPoolExecutor(max_workers=1)
        self.pending = None
        self.pending_done = None
        self.pending_is_poll = False
        self.queued = None
        self.close_requested = False
        self.closing = False
        self.draft = []
        self.selected = None
        self.live = []
        self.owned_count = 0
        self.geometry = None
        self.tool = tk.StringVar(value='select')
        self.status = tk.StringVar(value='CARLA 연결 확인 중…')
        self.notice = tk.StringVar(value='차량/보행자 추가를 선택하고 지도를 클릭하세요.')
        self.counts = tk.StringVar(value='주변 차량 —  ·  보행자 —')
        self.active = tk.StringVar(value='편집 중 · 아직 실행하지 않음')
        self.vehicle_count = tk.IntVar(value=75)
        self.walker_count = tk.IntVar(value=250)
        self.mode = tk.StringVar(value=MODE_LABELS['parked'])
        self.speed = tk.DoubleVar(value=0)
        self.yaw = tk.DoubleVar(value=0)
        self.position_label = tk.StringVar(value='선택한 객체가 없습니다.')
        self.speed_label = tk.StringVar(value='속도 (km/h)')
        self.cx, self.cy, self.scale = 0, 0, 1
        self.pan = None
        self.map_dirty = True
        self.loaded_once = False
        self._build()
        self._load_previous()
        self.submit(lambda: read_map('/home/a/autoware_data/maps/Town05/lanelet2_map.osm'), self.on_map)
        self.root.after(100, self.pump)
        self.root.protocol('WM_DELETE_WINDOW', self.close)

    def _build(self):
        r = self.root
        r.title('CARLA 시나리오 스튜디오 · Town05')
        r.geometry('1400x960')
        r.minsize(1080, 900)
        r.configure(bg=BG)
        r.option_add('*Font', ('Noto Sans CJK KR', 10))
        style = ttk.Style(r)
        style.theme_use('clam')
        style.configure('.', background=PANEL, foreground=TEXT, font=('Noto Sans CJK KR', 10))
        style.configure('TFrame', background=PANEL)
        style.configure('TLabel', background=PANEL, foreground=TEXT)
        style.configure('Muted.TLabel', foreground=MUTED)
        style.configure('Title.TLabel', font=('Noto Sans CJK KR', 18, 'bold'), background=BG)
        style.configure('Heading.TLabel', font=('Noto Sans CJK KR', 11, 'bold'))
        style.configure('TButton', padding=(10, 7), background='#2a3d4c', borderwidth=0)
        style.map('TButton', background=[('active', '#365568')])
        style.configure('Accent.TButton', background=ACCENT, foreground='#102d2a', font=('Noto Sans CJK KR', 10, 'bold'))
        style.map('Accent.TButton', background=[('active', '#79e3d2')])
        style.configure('TEntry', fieldbackground='#233545', foreground=TEXT, insertcolor=TEXT)
        style.configure('TSpinbox', fieldbackground='#233545', foreground=TEXT, arrowsize=16)
        style.configure('TCombobox', fieldbackground='#233545', foreground=TEXT, padding=5)
        style.map('TCombobox', fieldbackground=[('readonly', '#233545')], foreground=[('readonly', TEXT)])
        style.configure('Treeview', background='#14212d', fieldbackground='#14212d', foreground=TEXT, rowheight=34, borderwidth=0)
        style.configure('Treeview.Heading', background='#233545', foreground=MUTED)
        style.map('Treeview', background=[('selected', '#285869')])
        top = tk.Frame(r, bg=BG)
        top.pack(fill='x', padx=22, pady=(16, 14))
        ttk.Label(top, text='CARLA 시나리오 스튜디오', style='Title.TLabel').pack(side='left')
        tk.Label(top, textvariable=self.status, bg=BG, fg=ACCENT, font=('Noto Sans CJK KR', 10)).pack(side='right')
        body = ttk.Frame(r)
        body.pack(fill='both', expand=True, padx=18)
        left = ttk.Frame(body, width=245, padding=14)
        left.pack(side='left', fill='y')
        left.pack_propagate(False)
        center = tk.Frame(body, bg=BG)
        center.pack(side='left', fill='both', expand=True, padx=12)
        right = ttk.Frame(body, width=280, padding=14)
        right.pack(side='right', fill='y')
        right.pack_propagate(False)

        def heading(parent, text):
            ttk.Label(parent, text=text, style='Heading.TLabel').pack(anchor='w', pady=(0, 9))
        heading(left, '주변 교통량')
        ttk.Label(left, text='배경 차량', style='Muted.TLabel').pack(anchor='w')
        ttk.Spinbox(left, from_=0, to=300, textvariable=self.vehicle_count, width=12).pack(fill='x', pady=(3, 9))
        ttk.Label(left, text='전체 보행자', style='Muted.TLabel').pack(anchor='w')
        ttk.Spinbox(left, from_=0, to=800, textvariable=self.walker_count, width=12).pack(fill='x', pady=(3, 10))
        ttk.Button(left, text='교통량 적용', command=self.apply_traffic).pack(fill='x')
        ttk.Label(left, textvariable=self.counts, wraplength=215, style='Muted.TLabel').pack(anchor='w', pady=10)
        ttk.Separator(left).pack(fill='x', pady=10)
        heading(left, '지도에서 배치')
        for text, value in [('선택 / 지도 이동', 'select'), ('＋ 차량 추가', 'vehicle'), ('＋ 보행자 추가', 'pedestrian'), ('선택 보행자 도착 지점', 'target')]:
            ttk.Radiobutton(left, text=text, value=value, variable=self.tool, command=self.tool_changed).pack(anchor='w', pady=6)
        ttk.Label(left, text='차량은 차로에, 보행자는 보도에\n맞춰 배치됩니다.', style='Muted.TLabel').pack(anchor='w', pady=12)
        ttk.Button(left, text='전체 지도', command=self.fit).pack(fill='x', pady=3)
        ttk.Button(left, text='현재 주행 차량으로 이동', command=self.focus_ego).pack(fill='x', pady=3)
        ttk.Label(left, text='휠: 확대 / 축소\n선택 도구에서 빈 곳 드래그: 이동\n청록색: 편집 객체 · 노란색: 주행 차량', wraplength=220, style='Muted.TLabel').pack(anchor='w', pady=15)

        bar = tk.Frame(center, bg=BG)
        bar.pack(fill='x', pady=(2, 10))
        tk.Label(bar, text='TOWN05  /  SCENARIO MAP', bg=BG, fg=MUTED, font=('Noto Sans CJK KR', 10, 'bold')).pack(side='left')
        tk.Label(bar, textvariable=self.active, bg=BG, fg=ACCENT).pack(side='right')
        self.canvas = tk.Canvas(center, bg='#0c1721', highlightthickness=0)
        self.canvas.pack(fill='both', expand=True)
        self.canvas.bind('<Configure>', lambda _: self.invalidate_map())
        self.canvas.bind('<ButtonPress-1>', self.click)
        self.canvas.bind('<B1-Motion>', self.drag)
        self.canvas.bind('<ButtonRelease-1>', lambda _: setattr(self, 'pan', None))
        self.canvas.bind('<Button-4>', lambda e: self.zoom(e, 1.18))
        self.canvas.bind('<Button-5>', lambda e: self.zoom(e, 1/1.18))
        self.canvas.bind('<MouseWheel>', lambda e: self.zoom(e, 1.18 if e.delta > 0 else 1/1.18))
        self.canvas.create_text(280, 180, text='Town05 지도 읽는 중…', fill=MUTED)

        heading(right, '선택한 객체')
        ttk.Label(right, textvariable=self.position_label, style='Muted.TLabel', wraplength=246).pack(anchor='w', pady=(0, 10))
        ttk.Label(right, text='동작').pack(anchor='w')
        self.mode_box = ttk.Combobox(right, textvariable=self.mode, state='readonly', values=list(MODE_LABELS.values()))
        self.mode_box.pack(fill='x', pady=(3, 9))
        ttk.Label(right, textvariable=self.speed_label).pack(anchor='w')
        ttk.Spinbox(right, from_=0, to=60, increment=.5, textvariable=self.speed).pack(fill='x', pady=(3, 9))
        ttk.Label(right, text='방향 (도)').pack(anchor='w')
        ttk.Spinbox(right, from_=-180, to=180, increment=5, textvariable=self.yaw).pack(fill='x', pady=(3, 9))
        ttk.Button(right, text='객체 설정 반영', command=self.edit_selected).pack(fill='x')
        actions = ttk.Frame(right)
        actions.pack(fill='x', pady=8)
        ttk.Button(actions, text='위치 옮기기', command=lambda: self.tool.set('move')).pack(side='left', expand=True, fill='x')
        ttk.Button(actions, text='삭제', command=self.delete_selected).pack(side='right', padx=(6, 0))
        ttk.Separator(right).pack(fill='x', pady=10)
        heading(right, '시나리오 객체')
        self.tree = ttk.Treeview(right, columns=('type', 'mode'), show='headings', height=3, selectmode='browse')
        self.tree.heading('type', text='객체')
        self.tree.heading('mode', text='동작')
        self.tree.column('type', width=78, stretch=False)
        self.tree.column('mode', width=148)
        self.tree.pack(fill='both', expand=True)
        self.tree.bind('<<TreeviewSelect>>', self.tree_selected)
        filebar = ttk.Frame(right)
        filebar.pack(fill='x', pady=10)
        ttk.Button(filebar, text='저장', command=self.save).pack(side='left', expand=True, fill='x')
        ttk.Button(filebar, text='불러오기', command=self.load).pack(side='left', expand=True, fill='x', padx=(6, 0))
        ttk.Button(right, text='▶ 시나리오 실행', style='Accent.TButton', command=self.run).pack(fill='x', pady=(3, 7))
        ttk.Button(right, text='■ 실행 객체 정리', command=self.stop).pack(fill='x')
        ttk.Label(right, text='정리는 이 편집기의 객체만 삭제합니다.\n주행 차량과 경로는 유지합니다.', wraplength=247, style='Muted.TLabel').pack(anchor='w', pady=(10, 0))
        bottom = tk.Frame(r, bg=BG)
        bottom.pack(fill='x', padx=22, pady=12)
        tk.Label(bottom, textvariable=self.notice, bg=BG, fg=TEXT, anchor='w').pack(fill='x')

    def _load_previous(self):
        try:
            conf = json.loads(TRAFFIC_CONFIG.read_text())
            self.vehicle_count.set(conf['vehicles'])
            self.walker_count.set(conf['walkers'])
            draft_path = STATE/'draft.json'
            if draft_path.exists():
                data = validate_scenario(json.loads(draft_path.read_text()))
                self.draft = data['actors']
                self.refresh_list()
        except (ValueError, OSError, KeyError):
            self.notice.set('저장된 초안을 읽지 못해 빈 시나리오로 시작했습니다.')

    def document(self):
        return {'schema': 1, 'map': 'Town05_Opt', 'actors': self.draft}

    def autosave(self):
        # Incomplete walking destinations are a valid editing state; they cannot run.
        atomic_json(STATE/'draft.json', self.document())

    def submit(self, work, done=None):
        if self.pending is not None:
            if self.pending_is_poll and self.queued is None:
                self.queued = (work, done)
                return True
            self.notice.set('이전 작업이 끝날 때까지 잠시 기다려 주세요.')
            return False
        self.pending = self.executor.submit(work)
        self.pending_done = done
        self.pending_is_poll = False
        return True

    def pump(self):
        if self.pending is not None and self.pending.done():
            future, callback = self.pending, self.pending_done
            self.pending = self.pending_done = None
            self.pending_is_poll = False
            try:
                result = future.result()
                if callback:
                    callback(result)
            except Exception as exc:
                self.notice.set(str(exc))
                self.status.set('연결/작업 확인 필요')
                if self.closing:
                    self.closing = self.close_requested = False
                    messagebox.showerror('정리 실패', f'{exc}\n객체를 정리한 뒤 다시 닫아 주세요.', parent=self.root)
        if self.close_requested and not self.closing and self.pending is None:
            self.queued = None
            self.closing = True
            self.submit(self.engine.stop, self.notice.set)
        elif self.closing:
            if self.pending is None:
                self.executor.shutdown(wait=False)
                self.root.destroy()
                return
        elif self.pending is None and self.queued is not None:
            work, done = self.queued
            self.queued = None
            self.submit(work, done)
        elif not self.close_requested and self.pending is None and self.geometry is not None:
            self.pending = self.executor.submit(self.engine.status)
            self.pending_done = self.on_status
            self.pending_is_poll = True
        self.root.after(400, self.pump)

    def on_map(self, data):
        self.geometry = data
        self.fit()

    def on_status(self, data):
        self.live = data['actors']
        self.owned_count = data['owned']
        self.status.set(f"● 연결됨 · Town05 · {data['fps']:.1f} Hz")
        self.counts.set(f"현재 주변 차량 {sum(a['kind']=='vehicle' and not a['ego'] and not a['owned'] for a in self.live)}대\n현재 보행자 {sum(a['kind']=='pedestrian' for a in self.live)}명")
        self.active.set(f'실행 중 · {self.owned_count}개' if data['draft_active'] else f'편집 초안 · {len(self.draft)}개')
        self.draw()

    def invalidate_map(self):
        self.map_dirty = True
        self.draw()

    def fit(self):
        if not self.geometry:
            return
        points = [p for poly in self.geometry['roads'] for p in poly]
        xs, ys = zip(*points)
        self.cx, self.cy = (min(xs)+max(xs))/2, (min(ys)+max(ys))/2
        self.root.update_idletasks()
        self.scale = min((self.canvas.winfo_width()-70)/(max(xs)-min(xs)), (self.canvas.winfo_height()-70)/(max(ys)-min(ys)))
        self.invalidate_map()

    def focus_ego(self):
        ego = next((a for a in self.live if a['ego']), None)
        if ego:
            self.cx, self.cy, self.scale = ego['x'], ego['y'], 5
            self.invalidate_map()
        else:
            self.notice.set('현재 주행 차량의 위치를 기다리고 있습니다.')

    def screen(self, x, y):
        return self.canvas.winfo_width()/2 + (x-self.cx)*self.scale, self.canvas.winfo_height()/2 - (y-self.cy)*self.scale

    def world(self, x, y):
        return self.cx+(x-self.canvas.winfo_width()/2)/self.scale, self.cy-(y-self.canvas.winfo_height()/2)/self.scale

    def draw(self):
        if not self.geometry:
            return
        c = self.canvas
        if self.map_dirty:
            c.delete('all')
            for poly in self.geometry['roads']:
                coords = [v for p in poly for v in self.screen(*p)]
                c.create_polygon(coords, fill='#253646', outline='#456071', width=.6, tags='map')
            for poly in self.geometry['crossings']:
                c.create_polygon([v for p in poly for v in self.screen(*p)], fill='#526a72', outline='#73949a', tags='map')
            for line in self.geometry['stops']:
                c.create_line([v for p in line for v in self.screen(*p)], fill='#edc477', width=1.5, tags='map')
            self.map_dirty = False
        c.delete('actor')
        for a in self.live:
            x,y = self.screen(a['x'], a['y'])
            if a['ego']:
                self.draw_vehicle(a, '#f8ce65', 2)
                c.create_text(x,y-17,text='내 차량',fill='#f8ce65',tags='actor')
            elif a['owned']:
                self.draw_vehicle(a, '#41cfbc', 1) if a['kind']=='vehicle' else c.create_oval(x-3,y-3,x+3,y+3,fill=ACCENT,outline='',tags='actor')
            else:
                size = 2.4 if a['kind']=='vehicle' else 1.5
                c.create_oval(x-size,y-size,x+size,y+size,fill='#8d9ead' if a['kind']=='vehicle' else '#dc9a73',outline='',tags='actor')
        for i,a in enumerate(self.draft):
            x,y=self.screen(a['x'],a['y'])
            color='#b2ffeb' if a['id']==self.selected else ACCENT
            radius=8 if a['id']==self.selected else 6
            c.create_oval(x-radius,y-radius,x+radius,y+radius,outline=color,width=2,fill='#153d40',tags='actor')
            theta=math.radians(a['yaw'])
            c.create_line(x,y,x+14*math.cos(theta),y-14*math.sin(theta),fill=color,arrow='last',tags='actor')
            c.create_text(x+12,y-12,text=str(i+1),fill=color,tags='actor')
            if a.get('target'):
                tx,ty=self.screen(*a['target']);c.create_line(x,y,tx,ty,fill=ACCENT,dash=(4,4),arrow='last',tags='actor')
                c.create_oval(tx-4,ty-4,tx+4,ty+4,outline=ACCENT,tags='actor')
        x,y=20,c.winfo_height()-25
        c.create_line(x,y,x+20*self.scale,y,fill=MUTED,width=2,tags='actor')
        c.create_text(x,y-12,text='20 m',anchor='w',fill=MUTED,tags='actor')

    def draw_vehicle(self, a, color, width):
        theta=math.radians(a['yaw']);ct,st=math.cos(theta),math.sin(theta)
        coords=[]
        for dx,dy in ((2.3,1),(-2.3,1),(-2.3,-1),(2.3,-1)):
            coords.extend(self.screen(a['x']+dx*ct-dy*st,a['y']+dx*st+dy*ct))
        self.canvas.create_polygon(coords,fill=color,outline=color,width=width,tags='actor')

    def zoom(self, event, factor):
        before=self.world(event.x,event.y)
        self.scale=max(.3,min(25,self.scale*factor))
        after=self.world(event.x,event.y)
        self.cx+=before[0]-after[0];self.cy+=before[1]-after[1]
        self.invalidate_map()

    def click(self, event):
        if not self.geometry:
            return
        x,y=self.world(event.x,event.y);tool=self.tool.get()
        if tool=='select':
            near=sorted((math.dist(self.screen(a['x'],a['y']),(event.x,event.y)),a['id']) for a in self.draft)
            if near and near[0][0]<16:
                self.select(near[0][1]);return
            self.pan=(event.x,event.y,self.cx,self.cy);return
        if self.owned_count:
            self.notice.set('실행 객체를 정리한 뒤 배치를 변경하세요.');return
        selected=self.current()
        if tool=='target':
            if not selected or selected['kind']!='pedestrian':
                self.notice.set('목록에서 보행자를 먼저 선택하세요.');return
            selected['target']=[x,y];selected['mode']='walk';selected['speed']=max(selected['speed'],1.2)
            self.refresh_list();self.select(selected['id']);self.autosave();self.draw();self.notice.set('보행자의 이동 목표를 지정했습니다.');return
        if tool=='move' and not selected:
            self.notice.set('옮길 객체를 먼저 선택하세요.');return
        kind=selected['kind'] if tool=='move' else tool
        if kind not in ('vehicle','pedestrian'):
            return
        aid=selected['id'] if tool=='move' else None
        def placed(p):
            if aid:
                actor=next(a for a in self.draft if a['id']==aid);actor.update(p)
            else:
                actor=dict(id=uuid.uuid4().hex,kind=kind,mode='parked' if kind=='vehicle' else 'wait',speed=0 if kind=='vehicle' else 1.2,target=None,**p)
                self.draft.append(actor)
            self.refresh_list();self.select(actor['id']);self.autosave();self.draw()
            self.notice.set('편집 초안에 배치했습니다. 실행 버튼을 눌러 적용하세요.')
        self.submit(lambda:self.engine.snap(kind,x,y),placed)

    def drag(self,event):
        if self.pan and self.tool.get()=='select':
            x,y,cx,cy=self.pan;self.cx=cx-(event.x-x)/self.scale;self.cy=cy+(event.y-y)/self.scale;self.invalidate_map()

    def tool_changed(self):
        self.notice.set({'select':'객체 선택 또는 빈 곳 드래그로 지도를 이동하세요.','vehicle':'차량을 놓을 도로를 클릭하세요.','pedestrian':'보행자를 놓을 보도 근처를 클릭하세요.','target':'보행자를 선택한 뒤 도착 지점을 클릭하세요.'}.get(self.tool.get(),''))

    def current(self):
        return next((a for a in self.draft if a['id']==self.selected),None)

    def select(self,aid):
        self.selected=aid;a=self.current()
        if not a:return
        self.tree.selection_set(aid)
        self.mode_box.configure(values=[MODE_LABELS[m] for m in (('parked','autopilot') if a['kind']=='vehicle' else ('wait','walk'))])
        self.mode.set(MODE_LABELS[a['mode']]);self.speed.set(a['speed']);self.yaw.set(round(a['yaw'],1))
        self.speed_label.set('속도 (km/h)' if a['kind']=='vehicle' else '속도 (m/s)')
        self.position_label.set(f"{'차량' if a['kind']=='vehicle' else '보행자'} · X {a['x']:.1f} / Y {a['y']:.1f}")
        self.draw()

    def tree_selected(self,_):
        ids=self.tree.selection()
        if ids and ids[0]!=self.selected:self.select(ids[0])

    def refresh_list(self):
        self.tree.delete(*self.tree.get_children())
        for i,a in enumerate(self.draft):self.tree.insert('', 'end',iid=a['id'],values=(f"{i+1} {'차량' if a['kind']=='vehicle' else '보행자'}",MODE_LABELS[a['mode']]))

    def edit_selected(self):
        a=self.current()
        if not a:return
        if self.owned_count:self.notice.set('실행 객체를 정리한 뒤 설정을 변경하세요.');return
        try:
            updated=dict(a,mode=next(k for k,v in MODE_LABELS.items() if v==self.mode.get()),speed=self.speed.get(),yaw=self.yaw.get())
            validate_scenario({'schema':1,'map':'Town05_Opt','actors':[updated]})
            a.update(updated);self.refresh_list();self.select(a['id']);self.autosave();self.notice.set('초안의 객체 설정을 반영했습니다.')
        except (ValueError,tk.TclError,StopIteration) as exc:self.notice.set(str(exc))

    def delete_selected(self):
        if self.owned_count:self.notice.set('실행 객체를 정리한 뒤 초안을 수정하세요.');return
        self.draft=[a for a in self.draft if a['id']!=self.selected];self.selected=None;self.position_label.set('선택한 객체가 없습니다.');self.refresh_list();self.autosave();self.draw()

    def apply_traffic(self):
        try:v,w=self.vehicle_count.get(),self.walker_count.get()
        except tk.TclError:self.notice.set('차량과 보행자 수는 정수로 입력하세요.');return
        self.submit(lambda:self.engine.traffic(v,w),self.notice.set)

    def run(self):
        try:data=validate_scenario(json.loads(json.dumps(self.document())))
        except ValueError as exc:self.notice.set(str(exc));return
        self.submit(lambda:self.engine.start(data),self.notice.set)

    def stop(self):
        self.submit(self.engine.stop,self.notice.set)

    def save(self):
        try:validate_scenario(self.document())
        except ValueError as exc:self.notice.set(str(exc));return
        directory=ROOT/'scenarios/gui';directory.mkdir(parents=True,exist_ok=True)
        path=filedialog.asksaveasfilename(parent=self.root,title='시나리오 저장',initialdir=directory,defaultextension='.json',filetypes=[('시나리오 JSON','*.json')])
        if path:
            atomic_json(path,self.document());self.notice.set(f'저장했습니다: {Path(path).name}')

    def load(self):
        if self.owned_count:self.notice.set('실행 객체를 먼저 정리하세요.');return
        path=filedialog.askopenfilename(parent=self.root,title='시나리오 불러오기',initialdir=ROOT/'scenarios/gui',filetypes=[('시나리오 JSON','*.json')])
        if not path:return
        try:
            data=validate_scenario(json.loads(Path(path).read_text()));self.draft=data['actors'];self.selected=None;self.refresh_list();self.autosave();self.draw();self.notice.set('초안을 불러왔습니다. 실행 전 위치와 동작을 확인하세요.')
        except (ValueError,OSError) as exc:self.notice.set(str(exc))

    def close(self):
        self.autosave();self.notice.set('실행 객체를 정리하고 창을 닫는 중…');self.close_requested=True


def main():
    STATE.mkdir(parents=True,exist_ok=True)
    lock=(STATE/'gui.lock').open('w')
    try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:
        r=tk.Tk();r.withdraw();messagebox.showinfo('시나리오 스튜디오','이미 실행 중입니다. 열려 있는 창을 확인하세요.');r.destroy();return
    root=tk.Tk();ScenarioGUI(root);root.mainloop()


if __name__=='__main__':main()
