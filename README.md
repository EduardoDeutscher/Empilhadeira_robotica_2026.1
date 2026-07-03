# Projeto empilhadeira robótica 2026.1

Projeto desenvolvido para a disciplina ENG4061 – Projeto de Robótica  
Prof. João Ricardo | Turma 3VA

**Integrantes:**
- Eduardo Deutscher — 2210218
- Eric — 2210608
- Marcelo — 2220426
- Marius

## Descrição do Projeto

Empilhadeira robótica autônoma capaz de localizar paletes por meio de visão computacional (AprilTags), aproximar-se autonomamente utilizando o algoritmo Pure Pursuit e elevar a carga com um garfo motorizado. O sistema também suporta operação manual completa via dashboard web, acessível por qualquer dispositivo na mesma rede Wi-Fi.

## Arquitetura do Sistema

## Hardware Necessário 

| Componente                        | Quantidade |
|-----------------------------------|------------|
| Raspberry Pi 3                    | 1          |
| Arduino Mega 2560                 | 1          |
| Motor DC LEGO NXT 53787 (encoder) | 2          |
| Motor de passo NEMA 17            | 1          |
| Driver A4988                      | 1          |
| Ponte H L298N                     | 1          |
| Bateria Li-Ion 18650 3.7V 2000mAh | 3          |
| BMS 3S 20A                        | 1          |
| Regulador DC-DC Buck LM2596       | 1          |
| Câmera Multilaser 480p 30fps      | 1          |
| Capacitor 100µF 25V               | 1          |
| Barras de aço cilíndricas (guias) | 2          |
| Rolamentos lineares LM6UU/LM8UU   | 4          |
| Filamento PLA (~1kg)              | 1          |

## Dependências e Instalações 

### Raspberry Pi (Python)

```bash
pip install opencv-python-headless pupil-apriltags flask flask-sock pyserial
```

### Arduino IDE

Instalar via Library Manager:
- **AccelStepper** by Mike McCauley

Instalar via Boards Manager:
- **Arduino AVR Boards** (já incluso por padrão)

### Configuração da porta serial

Verificar a porta do Arduino no Pi antes de rodar:
```bash
ls /dev/tty*
```
Atualizar a linha no `servidor.py` se necessário:
```python
arduino = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
```

### Inicialização automática no Pi (opcional)

Para o servidor subir automaticamente ao ligar:
```bash
sudo nano /etc/rc.local
```
Adicionar antes do `exit 0`:
```bash
sleep 10
cd /home/pi && python servidor.py &
```

## Estrutura de Arquivos



## Modos de operação

### 🔍 Busca
Modo inicial. O robô avança, vira para a esquerda, retorna ao centro,
vira para a direita e repete o ciclo até encontrar uma AprilTag.

### 🎯 Pure Pursuit
Ativado ao detectar a AprilTag. O robô se aproxima e se alinha
com o alvo usando o algoritmo Pure Pursuit. Ao atingir distância
< 0,30 m e ângulo < 10°, muda automaticamente para modo manual.

### 🕹️ Manual
Controle total pelo dashboard web. Movimentação via botões (ou
teclado WASD) e controle do garfo (J/K). Para retornar ao modo
busca, pressionar o botão "Alternar Modo" no dashboard.

### Comandos seriais (Pi → Arduino)

| Comando  | Ação                  |
|----------|-----------------------|
| FRENTE   | Ambos motores frente  |
| TRAS     | Ambos motores trás    |
| ESQUERDA | Rotação para esquerda |
| DIREITA  | Rotação para direita  |
| PARAR    | Para os motores       |
| SUBIR    | Garfo sobe (10 steps) |
| DESCER   | Garfo desce (10 steps)|


## Ligação dos componentes

### Motores LEGO (tração) → Ponte H L298N → Arduino Mega

| Arduino Mega | Ponte H L298N |
|--------------|---------------|
| Pino 11      | IN1           |
| Pino 10      | IN2           |
| Pino 9       | IN3           |
| Pino 8       | IN4           |
| GND          | GND           |

Jumpers ENA e ENB mantidos no lugar (+5V fixo).  
Alimentação da ponte: pack de baterias 12V → terminal VMOT.

### Motor NEMA 17 (garfo) → Driver A4988 → Arduino Mega

| Arduino Mega | A4988  |
|--------------|--------|
| Pino 44      | STEP   |
| Pino 42      | DIR    |
| Pino 46      | ENABLE |
| Pino 48      | RESET  |
| Pino 50      | SLEEP  |
| GND          | GND lógico |
| 5V           | VDD    |

Alimentação do driver: pack de baterias 12V → VMOT + GND potência.  
Capacitor 100µF instalado em paralelo com VMOT e GND (o mais próximo possível do driver).

### Raspberry Pi → Arduino Mega

Conexão via cabo USB (Serial UART 115200 bps).  
Alimentação do Pi: regulador LM2596 (bateria 12V → 5V).
