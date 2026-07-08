from flask import Flask, render_template_string, Response
from flask_sock import Sock
import cv2
import numpy as np
from pupil_apriltags import Detector
import serial
import time
import threading
import json

# ══════════════════════════════════════════════════════════════════
# VARIÁVEIS GLOBAIS
# ══════════════════════════════════════════════════════════════════
PORTA_SERIAL          = '/dev/ttyACM0'
PORTA_SERIAL_1        = '/dev/ttyACM1'

TIMEOUT_BUSCA_SEG     = 300.0
TIMEOUT_PERDA_TAG_SEG = 1.5
DISTANCIA_PARAR       = 0.3
ANGULO_ALINHADO       = 3.0
DISTANCIA_SEGURANCA   = 0.25

K_ANGULAR             = 2.5
VEL_MAX               = 200
VEL_MIN               = 90    # velocidade mínima para vencer atrito estático

NUM_PULSOS_360   = 12
T_PULSO          = 0.5
T_PAUSA_LEITURA  = 1.20
T_FRENTE         = 1.0
VEL_GIRO_BUSCA   = 150
VEL_FRENTE_BUSCA = 180

# --- VARIÁVEIS PURE PURSUIT (STOP AND GO) ---
T_MOVIMENTO_PP   = 0.3  # Tempo que ele passa andando (segundos)
T_PAUSA_PP       = 0.6  # Tempo que ele fica parado lendo a câmera (segundos)
pp_fase          = "pausado"
pp_timer         = time.time()

L_CHASSI = 0.15
TAG_SIZE = 0.040

fx, fy = 818.035048081, 818.035048081
cx, cy = 362.333072996, 212.410243815

# ══════════════════════════════════════════════════════════════════
camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
dist_coeffs   = np.array([-0.245767, 3.025417, -0.019545, 0.005721, -7.190337])
at_detector   = Detector(families='tag25h9')

BUSCA        = "busca"
PURE_PURSUIT = "pure_pursuit"
MANUAL       = "manual"
PARADO       = "parado"

estado            = MANUAL
busca_fase        = "parado_lendo"
pulsos_dados      = 0
busca_timer       = time.time()
busca_inicio      = time.time()
tag_perdida_timer = None
ultimo_cmd        = ""
tag_alvo_id       = 0

lock              = threading.Lock()

# Frame compartilhado entre thread de visão e stream MJPEG
frame_global      = None
lock_frame        = threading.Lock()

# Telemetria compartilhada para o dashboard
telemetria = {
    "dist": 0.0,
    "angulo": 0.0,
    "x": 0.0,
    "z": 0.0,
    "tag_vista": False,
    "vel_esq": 0,
    "vel_dir": 0,
}

# ══════════════════════════════════════════════════════════════════
# SERIAL E WATCHDOG
# ══════════════════════════════════════════════════════════════════
arduino = None

def conectar_arduino():
    global arduino
    if arduino is not None:
        try:
            arduino.close()
        except:
            pass
    for porta in [PORTA_SERIAL, PORTA_SERIAL_1]:
        try:
            print(f"Tentando {porta}...")
            arduino = serial.Serial(porta, 115200, timeout=1)
            time.sleep(2)
            print(f"✅ Arduino conectado em {porta}!")
            return True
        except Exception as e:
            print(f"❌ Falha em {porta}: {e}")
    arduino = None
    return False

def enviar(cmd):
    """
    Função de envio não bloqueante. Falha rápido se a conexão cair, 
    liberando a variável `arduino` para a thread do watchdog assumir.
    """
    global arduino
    if arduino:
        try:
            arduino.write((cmd + '\n').encode('utf-8'))
        except serial.SerialException as e:
            print(f"⚠️ Erro serial: {e}. Desconectando Arduino...")
            try:
                arduino.close()
            except:
                pass
            arduino = None
        except Exception as e:
            print(f"⚠️ Erro genérico serial: {e}")

def watchdog_arduino():
    """
    Thread de background que garante a reconexão do Arduino
    sem bloquear a thread de visão (OpenCV).
    """
    global arduino
    while True:
        if arduino is None:
            print("🔄 Tentando reconectar ao Arduino em background...")
            conectar_arduino()
        time.sleep(2)

conectar_arduino()

# ══════════════════════════════════════════════════════════════════
# MOVIMENTO
# ══════════════════════════════════════════════════════════════════
def enviar_rodas(vel_esq, vel_dir):
    # Garante velocidade mínima para vencer o atrito (zona morta do motor)
    def aplicar_zona_morta(v):
        if v == 0:
            return 0
        return int(np.sign(v) * max(VEL_MIN, abs(v)))

    vel_esq = aplicar_zona_morta(int(np.clip(vel_esq, -VEL_MAX, VEL_MAX)))
    vel_dir = aplicar_zona_morta(int(np.clip(vel_dir, -VEL_MAX, VEL_MAX)))

    dir_esq = 1 if vel_esq > 0 else (-1 if vel_esq < 0 else 0)
    dir_dir = 1 if vel_dir > 0 else (-1 if vel_dir < 0 else 0)

    with lock:
        telemetria["vel_esq"] = vel_esq
        telemetria["vel_dir"] = vel_dir

    enviar(f"DE:{dir_esq},VE:{abs(vel_esq)},DD:{dir_dir},VD:{abs(vel_dir)}")

# ══════════════════════════════════════════════════════════════════
# PURE PURSUIT — MELHORADO
# ══════════════════════════════════════════════════════════════════
def pure_pursuit_para_rodas(x, z):
    distancia = np.sqrt(x**2 + z**2)
    angulo    = np.degrees(np.arctan2(x, z))

    # ── Fase 1: longe — Pure Pursuit clássico ──────────────────
    if distancia > DISTANCIA_PARAR:
        L = distancia
        curvatura = (2.0 * x) / (L**2)
        
        # 1. Ajuste da Rampa de Desaceleração:
        fator_dist = (L - DISTANCIA_PARAR) / 0.7
        v_norm     = np.clip(fator_dist, 0.45, 1.0) 

        w_norm       = v_norm * curvatura * K_ANGULAR
        
        vel_esq_norm = v_norm - w_norm * L_CHASSI / 2
        vel_dir_norm = v_norm + w_norm * L_CHASSI / 2

        # 2. Normalização:
        max_norm = max(abs(vel_esq_norm), abs(vel_dir_norm), 1.0)
        vel_esq_norm /= max_norm
        vel_dir_norm /= max_norm

        return int(vel_esq_norm * VEL_MAX), int(vel_dir_norm * VEL_MAX)

    # ── Fase 2: perto — só corrige ângulo girando no eixo ─────
    # if abs(angulo) < ANGULO_ALINHADO:
    #    return 0, 0   # alinhado — sinaliza chegada

    # 3. Ajuste do Giro no Próprio Eixo:
    vel_giro = int(np.clip(abs(angulo) * 3.0, VEL_MIN, VEL_MAX))

    if angulo > 0:
        return vel_giro, -vel_giro
    else:
        return -vel_giro, vel_giro

# ══════════════════════════════════════════════════════════════════
# MODO BUSCA
# ══════════════════════════════════════════════════════════════════
def atualizar_busca():
    global busca_fase, busca_timer, busca_inicio, estado, pulsos_dados

    if time.time() - busca_inicio > TIMEOUT_BUSCA_SEG:
        print("Timeout de busca excedido.")
        enviar("PARAR")
        with lock:
            estado = PARADO
        return

    decorrido = time.time() - busca_timer

    if busca_fase == "parado_lendo":
        enviar("PARAR")
        if decorrido > T_PAUSA_LEITURA:
            if pulsos_dados >= NUM_PULSOS_360:
                busca_fase  = "frente"
                busca_timer = time.time()
            else:
                busca_fase  = "girando"
                busca_timer = time.time()

    elif busca_fase == "girando":
        if decorrido > T_PULSO:
            enviar("PARAR") 
            pulsos_dados += 1
            busca_fase   = "parado_lendo"
            busca_timer  = time.time()
        else:
            enviar_rodas(-VEL_GIRO_BUSCA, VEL_GIRO_BUSCA)
    
    elif busca_fase == "frente":
        if decorrido > T_FRENTE:
            enviar("PARAR") # Evita que o robô dê um tranco para frente no frame de transição
            pulsos_dados = 0
            busca_fase   = "parado_lendo"
            busca_timer  = time.time()
        else:
            enviar_rodas(VEL_FRENTE_BUSCA, VEL_FRENTE_BUSCA)

def resetar_busca():
    global busca_fase, busca_timer, busca_inicio, pulsos_dados
    busca_fase   = "parado_lendo"
    busca_timer  = time.time()
    busca_inicio = time.time()
    pulsos_dados = 0

# ══════════════════════════════════════════════════════════════════
# THREAD DE VISÃO E RECONEXÃO DE CÂMERA
# ══════════════════════════════════════════════════════════════════
def conectar_camera():
    for idx in [0, 1, 2]:
        print(f"Tentando câmera {idx}...")
        temp = cv2.VideoCapture(idx)
        if temp.isOpened():
            ret, _ = temp.read()
            if ret:
                temp.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                temp.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                print(f"✅ Câmera {idx} conectada!")
                return temp
        temp.release()
        time.sleep(0.5)
    return None

def thread_visao():
    global estado, tag_perdida_timer, ultimo_cmd, tag_alvo_id, frame_global
    global pp_fase, pp_timer

    cap = conectar_camera()
    if cap is None:
        print("❌ Nenhuma câmera detectada na inicialização.")

    w_frame = 640
    h_frame = 480
    new_cam, _ = cv2.getOptimalNewCameraMatrix(
        camera_matrix, dist_coeffs, (w_frame, h_frame), 1, (w_frame, h_frame)
    )

    falhas_camera = 0

    while True:
        if cap is None or not cap.isOpened():
            print("❌ Câmera offline. Tentando reconectar...")
            cap = conectar_camera()
            if cap is None:
                time.sleep(1)
                continue
            else:
                falhas_camera = 0

        ret, frame = cap.read()
        
        if not ret:
            falhas_camera += 1
            if falhas_camera > 15:
                print("⚠️ Câmera travou. Reiniciando conexão de vídeo...")
                cap.release()
                cap = None
            time.sleep(0.01)
            continue
            
        falhas_camera = 0

        und  = cv2.undistort(frame, camera_matrix, dist_coeffs, None, new_cam)
        gray = cv2.cvtColor(und, cv2.COLOR_BGR2GRAY)

        results = at_detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=[new_cam[0,0], new_cam[1,1], new_cam[0,2], new_cam[1,2]],
            tag_size=TAG_SIZE
        )

        tag_detectada = False
        x, z = 0.0, 0.0

        with lock:
            alvo_atual = tag_alvo_id
            modo       = estado

        for r in results:
            cor = (0, 255, 0) if r.tag_id == alvo_atual else (128, 128, 128)
            cv2.polylines(und, [r.corners.astype(np.int32)], True, cor, 2)
            cv2.putText(und, f"ID:{r.tag_id}", (int(r.corners[0][0]), int(r.corners[0][1]) - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, cor, 1)

            if r.tag_id == alvo_atual:
                tag_detectada = True
                x = r.pose_t[0][0]
                z = r.pose_t[2][0]

        distancia = float(np.sqrt(x**2 + z**2)) if tag_detectada else 0.0
        angulo    = float(np.degrees(np.arctan2(x, z))) if tag_detectada else 0.0

        with lock:
            telemetria["dist"]      = round(distancia, 3)
            telemetria["angulo"]    = round(angulo, 1)
            telemetria["x"]         = round(float(x), 3)
            telemetria["z"]         = round(float(z), 3)
            telemetria["tag_vista"] = tag_detectada

        cor_modo = {
            BUSCA: (255,165,0), PURE_PURSUIT: (0,255,0),
            MANUAL: (0,165,255), PARADO: (0,0,255)
        }.get(modo, (255,255,255))

        cv2.putText(und, f"MODO: {modo.upper()} | ALVO: ID {alvo_atual}",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, cor_modo, 2)

        if tag_detectada:
            cv2.putText(und, f"Dist:{distancia:.2f}m  Ang:{angulo:.1f}deg  X:{x:.2f}  Z:{z:.2f}",
                        (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)

        h, w = und.shape[:2]
        cv2.line(und, (w//2, 0), (w//2, h), (50, 50, 50), 1)

        _, jpeg = cv2.imencode('.jpg', und, [cv2.IMWRITE_JPEG_QUALITY, 70])
        with lock_frame:
            frame_global = jpeg.tobytes()

        # Máquina de estados
        if modo == BUSCA:
            if tag_detectada:
                print(f"Tag {alvo_atual} encontrada! Pure Pursuit.")
                resetar_busca()
                with lock:
                    estado = PURE_PURSUIT
                    tag_perdida_timer = None
                    pp_fase = "pausado"
                    pp_timer = time.time()
            else:
                atualizar_busca()

        elif modo == PURE_PURSUIT:
            if not tag_detectada:
                if tag_perdida_timer is None:
                    tag_perdida_timer = time.time()
                elif time.time() - tag_perdida_timer > TIMEOUT_PERDA_TAG_SEG:
                    print("Tag perdida. Voltando para busca.")
                    enviar("PARAR")
                    resetar_busca()
                    with lock:
                        estado = BUSCA
                        tag_perdida_timer = None
            else:
                tag_perdida_timer = None
                distancia = np.sqrt(x**2 + z**2)
                angulo    = np.degrees(np.arctan2(x, z))

                if distancia < DISTANCIA_PARAR and abs(angulo) < ANGULO_ALINHADO:
                    print(f"Alinhado! Dist:{distancia:.2f}m Ang:{angulo:.1f}°. MANUAL.")
                    enviar("PARAR")
                    with lock:
                        estado = MANUAL
                else:
                    decorrido_pp = time.time() - pp_timer

                    if pp_fase == "movendo":
                        vel_esq, vel_dir = pure_pursuit_para_rodas(x, z)
                        cmd = f"{vel_esq},{vel_dir}"
                        if cmd != ultimo_cmd:
                            enviar_rodas(vel_esq, vel_dir)
                            ultimo_cmd = cmd
                        
                        if decorrido_pp > T_MOVIMENTO_PP:
                            pp_fase = "pausado"
                            pp_timer = time.time()
                            enviar("PARAR")
                            ultimo_cmd = "PARAR"
                            
                    elif pp_fase == "pausado":
                        if decorrido_pp > T_PAUSA_PP:
                            pp_fase = "movendo"
                            pp_timer = time.time()

        elif modo in (MANUAL, PARADO):
            pass

# ══════════════════════════════════════════════════════════════════
# FLASK
# ══════════════════════════════════════════════════════════════════
app  = Flask(__name__)
sock = Sock(app)

@app.route('/video')
def video():
    def gerar():
        while True:
            with lock_frame:
                f = frame_global
            if f is None:
                time.sleep(0.05)
                continue
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + f + b'\r\n')
            time.sleep(0.05)
    return Response(gerar(), mimetype='multipart/x-mixed-replace; boundary=frame')

HTML = """
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Dashboard Empilhadeira</title>
  <style>
    * { box-sizing:border-box; margin:0; padding:0; }
    body { background:#111; color:#fff; font-family:sans-serif;
           display:flex; flex-direction:column; align-items:center;
           padding:16px; gap:12px; }
    h2 { font-size:17px; }
    #modo { font-size:14px; padding:8px 20px; border-radius:99px;
            background:#333; font-weight:bold; }

    /* Feed de câmera */
    #camera-wrap { width:100%; max-width:480px; aspect-ratio:4/3;
                   background:#000; border-radius:8px; overflow:hidden;
                   border:2px solid #333; position:relative; }
    #camera-wrap img { width:100%; height:100%; object-fit:contain; }
    #cam-badge { position:absolute; top:6px; right:8px; font-size:11px;
                 background:rgba(0,0,0,0.6); padding:2px 8px;
                 border-radius:99px; color:#aaa; }
    #tag-badge { position:absolute; top:6px; left:8px; font-size:11px;
                 padding:2px 10px; border-radius:99px; font-weight:bold; }

    /* Telemetria */
    .telem { display:grid; grid-template-columns:repeat(4,1fr);
             gap:6px; width:100%; max-width:480px; }
    .tbox { background:#222; border-radius:6px; padding:8px 4px;
            text-align:center; }
    .tbox .val { font-size:18px; font-weight:bold; }
    .tbox .lbl { font-size:10px; color:#888; margin-top:2px; }

    .btn-modo { padding:11px 0; font-size:14px; border:none; border-radius:8px;
                background:#378ADD; color:#fff; cursor:pointer;
                width:100%; max-width:480px; }
    .btn-retomar { background:#1D9E75; }

    .config-panel { background:#1a1a1a; padding:10px 12px; border-radius:8px;
                    width:100%; max-width:480px; display:flex;
                    flex-direction:column; gap:8px; border:1px solid #333; }
    .config-panel label { font-size:12px; color:#aaa; }
    input[type="number"], input[type="range"] {
      width:100%; padding:5px; background:#2a2a2a; color:#fff;
      border:1px solid #444; border-radius:4px; }

    .grid { display:grid; grid-template-columns:repeat(3,75px);
            grid-template-rows:repeat(3,75px); gap:8px; }
    .btn { width:75px; height:75px; font-size:22px; border:none;
           border-radius:12px; background:#2a2a2a; color:#fff;
           cursor:pointer; touch-action:manipulation; user-select:none; }
    .btn:active { background:#555; }
    .empty { background:transparent !important; pointer-events:none; }

    .garfo { display:flex; gap:8px; width:100%; max-width:480px; }
    .btn-garfo { flex:1; height:58px; font-size:14px; border:none;
                 border-radius:10px; background:#e67e22; color:#fff;
                 cursor:pointer; touch-action:manipulation; user-select:none; }
    .btn-garfo:active { background:#d35400; }
    #status { font-size:11px; color:#666; }
  </style>
</head>
<body>

  <h2>🚜 Empilhadeira Autônoma</h2>
  <div id="modo">Modo: —</div>

  <div id="camera-wrap">
    <img src="/video" alt="Feed da câmera">
    <div id="cam-badge">📷 AO VIVO</div>
    <div id="tag-badge">—</div>
  </div>

  <div class="telem">
    <div class="tbox"><div class="val" id="t_dist">—</div><div class="lbl">Dist (m)</div></div>
    <div class="tbox"><div class="val" id="t_ang">—</div><div class="lbl">Ângulo (°)</div></div>
    <div class="tbox"><div class="val" id="t_ve">—</div><div class="lbl">Roda Esq</div></div>
    <div class="tbox"><div class="val" id="t_vd">—</div><div class="lbl">Roda Dir</div></div>
  </div>

  <button class="btn-modo" onclick="cmd('ALTERNAR')">⏯ Alternar Modo Autônomo / Manual</button>
  <button class="btn-modo btn-retomar" id="btn-retomar"
          onclick="cmd('RETOMAR')" style="display:none">▶ Retomar Busca</button>

  <div class="config-panel">
    <label>ID da AprilTag Alvo:
      <input type="number" id="tag_id_input" value="0" min="0" max="100"
             onchange="atualizarTagId()">
    </label>
    <label>Velocidade Manual: <span id="vel_label">140</span>
      <input type="range" id="slider" min="60" max="220" value="140"
             oninput="document.getElementById('vel_label').textContent=this.value">
    </label>
  </div>

  <div class="grid">
    <div class="btn empty"></div>
    <button class="btn" ontouchstart="mv(1,1)"   ontouchend="stop()"
                        onmousedown="mv(1,1)"    onmouseup="stop()">▲</button>
    <div class="btn empty"></div>

    <button class="btn" ontouchstart="mv(-1,1)"  ontouchend="stop()"
                        onmousedown="mv(-1,1)"   onmouseup="stop()">◀</button>
    <button class="btn" onclick="stop()">⏹</button>
    <button class="btn" ontouchstart="mv(1,-1)"  ontouchend="stop()"
                        onmousedown="mv(1,-1)"   onmouseup="stop()">▶</button>

    <div class="btn empty"></div>
    <button class="btn" ontouchstart="mv(-1,-1)" ontouchend="stop()"
                        onmousedown="mv(-1,-1)"  onmouseup="stop()">▼</button>
    <div class="btn empty"></div>
  </div>

  <div class="garfo">
    <button class="btn-garfo" ontouchstart="cmd('SUBIR')"  ontouchend="cmd('PARAR_GARFO')"
                              onmousedown="cmd('SUBIR')"  onmouseup="cmd('PARAR_GARFO')">
      ⬆ Garfo</button>
    <button class="btn-garfo" ontouchstart="cmd('DESCER')" ontouchend="cmd('PARAR_GARFO')"
                              onmousedown="cmd('DESCER')" onmouseup="cmd('PARAR_GARFO')">
      ⬇ Garfo</button>
  </div>

  <div id="status">Conectando...</div>

  <script>
    const ws = new WebSocket('ws://' + location.host + '/ws');

    ws.onopen  = () => document.getElementById('status').textContent = 'Conectado ✓';
    ws.onclose = () => document.getElementById('status').textContent = 'Desconectado ✗';

    ws.onmessage = (e) => {
      const data = JSON.parse(e.data);

      if (data.modo) {
        const cores = { busca:'#7A4800', pure_pursuit:'#1D6B3A',
                        manual:'#1D3A80', parado:'#6B1D1D' };
        const el = document.getElementById('modo');
        el.textContent      = 'Modo: ' + data.modo.toUpperCase();
        el.style.background = cores[data.modo] || '#333';

        document.getElementById('btn-retomar').style.display =
          data.modo === 'parado' ? 'block' : 'none';
      }

      // Telemetria
      if (data.telem) {
        const t = data.telem;
        document.getElementById('t_dist').textContent = t.dist.toFixed(2);
        document.getElementById('t_ang').textContent  = t.angulo.toFixed(1);
        document.getElementById('t_ve').textContent   = t.vel_esq;
        document.getElementById('t_vd').textContent   = t.vel_dir;

        const badge = document.getElementById('tag-badge');
        if (t.tag_vista) {
          badge.textContent      = '🎯 TAG DETECTADA';
          badge.style.background = 'rgba(0,180,0,0.8)';
          badge.style.color      = '#fff';
        } else {
          badge.textContent      = '🔍 Buscando...';
          badge.style.background = 'rgba(200,100,0,0.7)';
          badge.style.color      = '#fff';
        }
      }
    };

    function vel() { return parseInt(document.getElementById('slider').value); }
    function mv(de, dd) { cmd('RODAS:' + (de * vel()) + ',' + (dd * vel())); }
    function stop() { cmd('PARAR'); }
    function cmd(c) { if (ws.readyState === 1) ws.send(JSON.stringify({cmd: c})); }

    function atualizarTagId() {
      const id = parseInt(document.getElementById('tag_id_input').value) || 0;
      if (ws.readyState === 1) ws.send(JSON.stringify({set_tag_id: id}));
    }

    // Teclado
    let keys = {};
    document.addEventListener('keydown', e => {
      if (keys[e.key]) return;
      keys[e.key] = true;
      if (e.key === 'w') mv( 1,  1);
      if (e.key === 's') mv(-1, -1);
      if (e.key === 'a') mv(-1,  1);
      if (e.key === 'd') mv( 1, -1);
      if (e.key === 'j') cmd('SUBIR');
      if (e.key === 'k') cmd('DESCER');
    });

    document.addEventListener('keyup', e => {
      delete keys[e.key];
      if (['w','s','a','d'].includes(e.key)) stop();
      if (['j','k'].includes(e.key)) cmd('PARAR_GARFO');
    });
  </script>
</body>
</html>
"""

@app.route('/')
def index():
    return render_template_string(HTML)

@sock.route('/ws')
def websocket(ws):
    global estado, tag_alvo_id

    with lock:
        ws.send(json.dumps({"modo": estado, "tag_id": tag_alvo_id}))

    ultimo_envio_telem = 0

    while True:
        try:
            msg = ws.receive(timeout=0.1)
        except:
            break

        agora = time.time()
        if agora - ultimo_envio_telem > 0.1:
            with lock:
                telem_snap = dict(telemetria)
                modo_snap  = estado
            try:
                ws.send(json.dumps({"modo": modo_snap, "telem": telem_snap}))
            except:
                break
            ultimo_envio_telem = agora

        if msg is None:
            continue

        data = json.loads(msg)

        if 'set_tag_id' in data:
            with lock:
                tag_alvo_id = int(data['set_tag_id'])
            print(f"Tag alvo: {tag_alvo_id}")
            continue

        c = data.get('cmd', '')
        with lock:
            modo = estado

        if c == 'ALTERNAR':
            with lock:
                if estado in (MANUAL, PURE_PURSUIT):
                    resetar_busca()
                    estado = BUSCA
                elif estado == BUSCA:
                    enviar("PARAR")
                    estado = MANUAL
                elif estado == PARADO:
                    resetar_busca()
                    estado = BUSCA

        elif c == 'RETOMAR':
            with lock:
                if estado == PARADO:
                    resetar_busca()
                    estado = BUSCA

        elif modo == MANUAL:
            if c == 'PARAR':
                enviar("PARAR")
            elif c == 'PARAR_GARFO':
                enviar("PARAR_GARFO")
            elif c.startswith('RODAS:'):
                partes  = c[6:].split(',')
                enviar_rodas(int(partes[0]), int(partes[1]))
            elif c == 'SUBIR':
                enviar('SUBIR')
            elif c == 'DESCER':
                enviar('DESCER')

if __name__ == "__main__":
    t_visao = threading.Thread(target=thread_visao, daemon=True)
    t_visao.start()

    t_arduino = threading.Thread(target=watchdog_arduino, daemon=True)
    t_arduino.start()

    app.run(host='0.0.0.0', port=5000)
