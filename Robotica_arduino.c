#include <AccelStepper.h>

#if !defined(__AVR_ATmega2560__)
#error "Selecione Arduino Mega 2560 em Ferramentas > Placa."
#endif

// ── Pinos Motor LEGO ────────────────────────────────────────────
const int IN1 = 11;
const int IN2 = 10;
const int ENB = 13;

const int IN3 = 9;
const int IN4 = 8;
const int ENA = 12;

// ── Pinos Motor de Passo ─────────────────────────────────────────
const int pino_step = 4;
const int pino_dir = 5;
const int pino_enable = 3;
const int pino_vcc = 6;

// ── Parâmetros ───────────────────────────────────────────────────
const int PASSOS_POR_TECLA = 180;
const int ALTURA_MAX_PASSOS = 3000;
const unsigned long WATCHDOG_MS = 3000;

// ── Estado ───────────────────────────────────────────────────────
AccelStepper garfo(1, pino_step, pino_dir);
String buffer = "";
int passos_acumulados = 0;
unsigned long ultimo_recebimento = 0;

// ── Motores LEGO ─────────────────────────────────────────────────
// Raspberry manda: "DE:1,VE:150,DD:1,VD:150"
// DE/DD: direção esq/dir  (1=frente, -1=ré, 0=parado)
// VE/VD: velocidade esq/dir (0-180, já calculado pelo Pi)
void moverRodas(int dir_esq, int vel_esq, int dir_dir, int vel_dir) {
  // Motor A — esquerda
  if (dir_esq > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else if (dir_esq < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }

  // Motor B — direita
  if (dir_dir > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else if (dir_dir < 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }

  // Aplica PWM nos dois ao mesmo tempo (minimiza delay entre rodas)
  analogWrite(ENA, vel_esq);
  analogWrite(ENB, vel_dir);
}

void parar() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

// ── Motor de Passo ───────────────────────────────────────────────
void garfoSubir(int passos) {
  if (passos_acumulados + passos > ALTURA_MAX_PASSOS) {
    Serial.println("LIMITE: garfo no topo.");
    return;
  }
  digitalWrite(pino_enable, HIGH);
  garfo.move(passos);
  while (garfo.distanceToGo() != 0) garfo.run();
  digitalWrite(pino_enable, LOW);
  passos_acumulados += passos;
  Serial.println("Garfo subiu. Pos: " + String(passos_acumulados));
}

void garfoDescer(int passos) {
  if (passos_acumulados - passos < 0) {
    Serial.println("LIMITE: garfo no fundo.");
    return;
  }
  digitalWrite(pino_enable, HIGH);
  garfo.move(-passos);
  while (garfo.distanceToGo() != 0) garfo.run();
  digitalWrite(pino_enable, LOW);
  passos_acumulados -= passos;
  Serial.println("Garfo desceu. Pos: " + String(passos_acumulados));
}

// ── Parser ───────────────────────────────────────────────────────
// Formatos recebidos:
//   "DE:1,VE:150,DD:1,VD:150\n"   → move rodas
//   "PARAR\n"                      → para tudo
//   "SUBIR\n"                      → garfo sobe
//   "DESCER\n"                     → garfo desce
void executarComando(String cmd) {
  ultimo_recebimento = millis();

  if (cmd == "PARAR") {
    parar();

  } else if (cmd == "SUBIR") {
    garfoSubir(PASSOS_POR_TECLA);

  } else if (cmd == "DESCER") {
    garfoDescer(PASSOS_POR_TECLA);

  } else if (cmd.startsWith("DE:")) {
    // Extrai os 4 valores
    // Formato: DE:1,VE:150,DD:1,VD:150
    int i0 = cmd.indexOf("DE:") + 3;
    int i1 = cmd.indexOf(",VE:");
    int j1 = i1 + 4;
    int i2 = cmd.indexOf(",DD:");
    int j2 = i2 + 4;
    int i3 = cmd.indexOf(",VD:");
    int j3 = i3 + 4;

    int dir_esq = cmd.substring(i0, i1).toInt();
    int vel_esq = cmd.substring(j1, i2).toInt();
    int dir_dir = cmd.substring(j2, i3).toInt();
    int vel_dir = cmd.substring(j3).toInt();

    moverRodas(dir_esq, vel_esq, dir_dir, vel_dir);

  } else {
    Serial.println("Cmd desconhecido: " + cmd);
  }
}

// ── Setup ────────────────────────────────────────────────────────
void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(pino_vcc, OUTPUT);
  pinMode(pino_enable, OUTPUT);

  digitalWrite(pino_vcc, HIGH);
  digitalWrite(pino_enable, LOW);

  parar();

  garfo.setMaxSpeed(800.0);
  garfo.setAcceleration(400.0);

  Serial.begin(115200);
  ultimo_recebimento = millis();
  Serial.println("Pronto.");
}

// ── Loop ─────────────────────────────────────────────────────────
void loop() {
  if (millis() - ultimo_recebimento > WATCHDOG_MS) {
    parar();
  }

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      buffer.trim();
      if (buffer.length() > 0) executarComando(buffer);
      buffer = "";
    } else {
      buffer += c;
    }
  }
}

