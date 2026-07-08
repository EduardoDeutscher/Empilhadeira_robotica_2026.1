# Projeto Empilhadeira Robótica 2026.1

Projeto desenvolvido para a disciplina ENG4061 – Projeto de Robótica  
Prof. João Ricardo | Turma 3VA

**Integrantes:**
- Eduardo Deutscher — 2210218
- Eric Siemsen — 2210608
- Marcelo Bessa — 2220426
- Marius

---

## Descrição do Projeto

Empilhadeira robótica autônoma capaz de localizar paletes por meio de visão computacional utilizando AprilTags. O sistema realiza uma busca ativa por uma tag específica e, ao identificá-la, utiliza o algoritmo Pure Pursuit para se locomover autonomamente até o objetivo. Além do modo autônomo, o robô suporta operação manual completa, permitindo o controle em tempo real da movimentação e do garfo motorizado para elevação da carga. A transição entre os modos e o controle manual são realizados via dashboard web, acessível por qualquer dispositivo conectado à mesma rede Wi-Fi, sem necessidade de instalar nenhum aplicativo.

---

## Arquitetura do Sistema

              
    Câmera ⭢ Raspberry Pi ⭢ Arduino ⭢ Motores
                   ⭡
               Dashboard

O Raspberry Pi é responsável por toda a lógica de alto nível: visão computacional, máquina de estados (Busca → Pure Pursuit → Manual) e servidor web Flask com WebSockets. O Arduino Mega executa apenas os comandos recebidos via serial, acionando os motores com PWM em tempo real. A comunicação entre os dois é feita por cabo USB (Serial UART 115200 bps).

---

## Hardware Necessário

| Componente                        | Quantidade |
|-----------------------------------|------------|
| Raspberry Pi 3                    | 1          |
| Arduino Mega 2560                 | 1          |
| Motor DC LEGO NXT 53787 (encoder) | 2          |
| Motor de passo NEMA 17            | 1          |
| Driver A4988                      | 1          |
| Ponte H L298N                     | 1          |
| Bateria Li-Ion 18650 3.7V         | 3          |
| BMS 3S 40A                        | 1          |
| Regulador DC-DC Buck LM2596       | 1          |
| Câmera Multilaser 480p 30fps      | 1          |
| Barras de aço cilíndricas (guias) | 2          |
| Rolamentos lineares LM6UU/LM8UU   | 4          |
| Filamento PLA (~1kg)              | 1          |

---

---

## Dependências e Instalação

### Raspberry Pi

Crie um ambiente virtual para evitar conflitos com o sistema:

```bash
python3 -m venv ~/venv
source ~/venv/bin/activate
pip install flask flask-sock opencv-python-headless pupil-apriltags pyserial numpy
```

> Para rodar em sessões futuras, sempre ative o ambiente antes:
> ```bash
> source ~/venv/bin/activate
> ```

### Arduino IDE

Instalar via **Library Manager**:
- `AccelStepper` 

---

## Como Rodar

### 1. Subir o sketch no Arduino

- Abra `Robotica_arduino.c` na Arduino IDE
- Selecione a placa: **Ferramentas → Placa → Arduino Mega 2560**
- Selecione a porta correta em **Ferramentas → Porta**
- Clique em **Upload**

### 2. Conectar o Arduino ao Raspberry Pi via USB

O código tenta automaticamente `/dev/ttyACM0` e `/dev/ttyACM1`. Para verificar qual porta está sendo usada:

```bash
ls /dev/tty*
# antes e depois de conectar o cabo — a porta nova é a do Arduino
```

### 3. Rodar o servidor no Raspberry Pi

```bash
source ~/venv/bin/activate
python Robotica_raspberryPi.py
```

O terminal mostrará o IP e a porta ao iniciar. Aguarde as mensagens:
```bash
✅ Arduino conectado em /dev/ttyACM0!
✅ Câmera 0 conectada!
```

---

### 4. Acessar o dashboard

No celular ou notebook conectado na **mesma rede Wi-Fi** que o Pi, abra o navegador e acesse: `http://[Endereço IP]:5000` 

---

## Inicialização Automática (opcional)

Para o servidor subir automaticamente ao ligar o Pi, sem precisar de teclado ou SSH:

```bash
sudo nano /etc/rc.local
```

Adicione antes do `exit 0`:

```bash
sleep 15
source /home/pi/venv/bin/activate && cd /home/pi && python Robotica_raspberryPi.py &
```

Salve, reinicie o Pi e aguarde ~15 segundos após ligar para acessar o dashboard.

---

## Modos de Operação

### 🔍 Busca
Modo inicial autônomo. O robô para, faz um giro completo de 360° em pulsos (pausa entre cada pulso para a câmera ler sem desfoque), procurando a AprilTag alvo. Se não encontrar após a volta completa, avança um pouco para frente e repete. Se o timeout de busca for atingido (padrão: 300s), entra em modo **Parado**.

### 🎯 Pure Pursuit
Ativado ao detectar a AprilTag alvo. Opera em dois regimes:
- **Longe da tag:** Pure Pursuit clássico — calcula curvatura e decompõe em velocidades das rodas via cinemática diferencial, com rampa de desaceleração proporcional à distância.
- **Perto da tag** (< `DISTANCIA_PARAR`): para a translação e corrige apenas o ângulo girando no próprio eixo, com velocidade proporcional ao erro angular.

Ao atingir distância < 0,30 m, muda automaticamente para **Manual**.

### 🕹️ Manual
Controle total pelo dashboard. WASD para movimentação, J/K para o garfo. O slider de velocidade define a intensidade. Para retornar ao modo Busca, pressionar **Alternar Modo** no dashboard.

### ⏸️ Parado
Estado de segurança atingido por timeout de busca. Pressionar **Retomar Busca** no dashboard para reiniciar.

---

## Protocolo Serial (Pi → Arduino)

| Comando                        | Ação                                        |
|-------------------------------|---------------------------------------------|
| `DE:1,VE:150,DD:1,VD:150\n`  | Move rodas com direção e velocidade PWM      |
| `PARAR\n`                     | Para todos os motores                        |
| `SUBIR\n`                     | Garfo sobe (N passos definido no Arduino)    |
| `DESCER\n`                     | Garfo desce (N passos definido no Arduino)  |
| `PARAR_GARFO\n`               | Para o motor de passo                        |

Formato do comando de rodas: `DE` = direção esquerda (1/-1/0), `VE` = velocidade esquerda (0–200), `DD` = direção direita, `VD` = velocidade direita. Todos os valores são calculados pelo Pi; o Arduino apenas executa.

---

## Parâmetros Ajustáveis

Todos no topo de `Robotica_raspberryPi.py`:

| Variável             | Padrão | Descrição                                              |
|----------------------|--------|--------------------------------------------------------|
| `TAG_ALVO`           | `0`    | ID da AprilTag a ser buscada (também ajustável no dashboard em tempo real) |
| `DISTANCIA_PARAR`    | `0.30` | Distância em metros para considerar "chegou"           |
| `ANGULO_ALINHADO`    | `3.0`  | Graus de tolerância para considerar "alinhado"         |
| `VEL_MAX`            | `200`  | PWM máximo enviado aos motores (0–255)                 |
| `VEL_MIN`            | `90`   | PWM mínimo para vencer atrito estático                 |
| `NUM_PULSOS_360`     | `12`   | Número de pulsos para completar 360° na busca          |
| `T_PULSO`            | `0.5`  | Duração de cada pulso de giro (segundos)               |
| `T_PAUSA_LEITURA`    | `1.20` | Pausa entre pulsos para leitura da câmera (segundos)   |
| `TIMEOUT_BUSCA_SEG`  | `300`  | Tempo máximo em busca antes de parar                   |
| `K_ANGULAR`          | `2.5`  | Ganho da velocidade angular no Pure Pursuit            |
| `L_CHASSI`           | `0.15` | Distância entre rodas em metros (bitola)               |

---

## Ligação dos Componentes

### Motores LEGO (tração) → Ponte H L298N → Arduino Mega

| Arduino Mega | Ponte H |
|--------------|---------|
| Pino 11      | IN1     |
| Pino 10      | IN2     |
| Pino 13      | ENB     |
| Pino 9       | IN3     |
| Pino 8       | IN4     |
| Pino 12      | ENA     |
| GND          | GND     |

Alimentação da ponte: pack de baterias 12V → terminal VMOT.

### Motor NEMA 17 (garfo) → Driver A4988 → Arduino Mega

| Arduino Mega | A4988   |
|--------------|---------|
| Pino 4       | STEP    |
| Pino 5       | DIR     |
| Pino 3       | ENABLE  |
| Pino 6       | VCC     |
| GND          | GND lógico |
| 5V           | VDD     |

Capacitor 100µF instalado em paralelo com VMOT e GND, o mais próximo possível do driver.  
Alimentação do driver: pack de baterias 12V → VMOT.

### Raspberry Pi → Arduino Mega

Conexão via cabo USB (Serial UART 115200 bps).  
Alimentação do Pi: regulador LM2596 (bateria 12V → 5V).

---

## Solução de Problemas Comuns

**Arduino não conecta:**
```bash
ls /dev/tty*  # verificar porta disponível
```
O código tenta `/dev/ttyACM0` e `/dev/ttyACM1` automaticamente. Tem reconexão automática em background se o cabo for desconectado.

**Câmera não abre:**
```bash
ls /dev/video*  # verificar índice disponível
```
O código tenta os índices 0, 1 e 2 automaticamente.

**Erro `externally-managed-environment` no pip:**
```bash
python3 -m venv ~/venv
source ~/venv/bin/activate
pip install flask flask-sock opencv-python-headless pupil-apriltags pyserial numpy
```

**Dashboard não abre pelo nome:**
Use o IP direto. Descobrir com `hostname -I` no Pi.

**Motores não reagem:**
Verificar baud rate no Serial Monitor (deve ser 115200) e confirmar que GND da fonte está conectado ao GND do Arduino.

---
