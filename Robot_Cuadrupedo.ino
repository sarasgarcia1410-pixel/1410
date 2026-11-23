/* ============================================================================
 *  ROBOT CUADRUPEDO  -  8 servos (2 DOF por pata)  -  IK real
 *  ESP32 + PCA9685 + Adafruit_PWMServoDriver
 * ----------------------------------------------------------------------------
 *
 *  TOPOLOGIA REAL DEL ROBOT (segun las fotos):
 *     CADERA (hombro) : eje lateral, mueve el femur ADELANTE / ATRAS
 *     RODILLA (brazo) : eje lateral, DOBLA la tibia (sube / baja el pie)
 *  => La pata es un mecanismo planar de 2 eslabones. Aqui se resuelve por
 *     cinematica inversa (IK), por eso el pie pisa plano y no arrastra.
 *
 *  MARCHA: CRAWL (ola). Una pata en el aire cada vez, las otras 3 empujan
 *  el cuerpo hacia atras -> el robot avanza recto. Orden: FL -> RR -> FR -> RL.
 *
 *  AL ENCENDER: se pone en pose neutra, espera 1.5 s y empieza a caminar solo.
 *
 *  CONSOLA SERIE (115200 baudios) para calibrar sin recompilar:
 *     ?                      ayuda
 *     g                      caminar        s    parar / quedarse de pie
 *     t <pata>               probar 1 pata (muestra el ciclo de esa pata)
 *     m <canal> <grados>     mover un servo suelto (calibracion)
 *     i <pata> <x> <y>       llevar el pie a una posicion (mm) con IK
 *     f <pata> h|k           invertir signo del hombro o de la rodilla
 *     x <pata> <mm>          desplazar el pie de esa pata adelante/atras
 *     geo <L1> <L2> <alto>   medidas en mm (femur, tibia, altura cadera)
 *     p                      mostrar configuracion actual
 *     d                      volcar la config como codigo C++ para pegar
 *
 *  patas: 0=FL delantera izq | 1=RR trasera der | 2=FR delantera der | 3=RL trasera izq
 * ==========================================================================*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

/* ------------------------------ I2C -------------------------------------- */
static const uint8_t PIN_SDA  = 21; //datos
static const uint8_t PIN_SCL  = 22; //reloj
static const uint8_t PCA_ADDR = 0x40; //direccion

/* --------------------- CALIBRACION DE PULSOS ----------------------------- */
static const int  SERVOMIN = 110;      // ticks -> ~0 grados   (MG90S)
static const int  SERVOMAX = 500;      // ticks -> ~180 grados (MG90S)

/* ============================ MEDIDAS (mm) =============================== */
/*  MIDELO CON REGLA, de eje a eje:
 *    L1      cadera -> rodilla   (largo del femur / brazo curvo)
 *    L2      rodilla -> punta del pie (largo de la tibia)
 *    BODY_H  altura del eje de cadera sobre el suelo cuando esta de pie
 *  Si BODY_H no coincide con la postura real, las patas quedaran mal
 *  repartidas: ajustalo hasta que el robot de pie quede como en las fotos.
 */
float L1     = 82.0f;
float L2     = 67.0f;
float BODY_H = 80.0f;

/* ============================ MARCHA ===================================== */
float    STEP_LEN   = 17.0f;   // largo total del paso (mm)
float    STEP_H     = 30.0f;   // altura a la que levanta el pie (mm)
uint32_t CYCLE_MS   = 2600;    // duracion de un ciclo de 4 patas (ms)
uint32_t RAMP_MS    = 1500;    // arranque suave
const float SWING_FRAC = 0.25f;

/* ============================ LAS PATAS ================================== */
/*  hipCh / kneeCh : canales de la PCA9685
 *  hipSign         : +1 / -1  (si el femur va al reves, se invierte)
 *  kneeSign        : +1 / -1  (si la rodilla dobla al reves, se invierte)
 *  hipNeutral      : angulo del servo del hombro en la pose de pie
 *  kneeNeutral     : angulo del servo de la rodilla en la pose de pie
 *  xOffset         : cuanto adelanta/atrasa el pie de esa pata (mm)
 */
struct Leg {
  uint8_t hipCh, kneeCh;
  int8_t  hipSign, kneeSign;
  float   hipNeutral, kneeNeutral;
  float   xOffset;
  const char *name;
};

Leg legs[4] = {                     //hombro
  // hipCh kneeCh hipSign kneeSign hipNeutral kneeNeutral xOffset  name
  {   1,    6,      -1,      +1,       90.0f,      90.0f,    0.0f, "DI" },
  {   2,    5,      +1,      -1,       60.0f,      90.0f,    0.0f, "TD" },
  {   3,    7,      +1,      -1,       90.0f,      90.0f,    0.0f, "DD" },
  {   0,    4,      -1,      +1,      120.0f,      90.0f,    0.0f, "TI" },
};

/*  Direccion de doblado de la rodilla:
 *    -1 = rodilla hacia atras (como pata trasera de perro)
 *    +1 = rodilla hacia delante (parece ser tu caso, segun las fotos)
 *  Si al caminar las patas se cruzan o se meten bajo el cuerpo, cambia a -1.
 */
int8_t KNEE_DIR = +1;

/* ========================= internos ====================================== */
#define RAD2DEG  57.29578f
#define NUM_LEGS 4

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA_ADDR);

float appliedAngle[16];
unsigned long t0 = 0;
unsigned long testT0 = 0;
uint8_t testLeg = 0;

enum Mode { M_WALK, M_STAND, M_MANUAL, M_TEST };
Mode mode = M_WALK;

float th1_0[4], th2_0[4];      // angulos de referencia de la pose neutra

/* ------------------------------------------------------------------------ */

void servoWrite(uint8_t ch, float angle) {
  if (angle < 0.0f)   angle = 0.0f;
  if (angle > 180.0f) angle = 180.0f;

  if (fabsf(angle - appliedAngle[ch]) < 0.3f) return;
  appliedAngle[ch] = angle;

  uint16_t pulse = (uint16_t)(SERVOMIN +
                   (SERVOMAX - SERVOMIN) * (angle / 180.0f));
  pwm.setPWM(ch, 0, pulse);
}

/*  IK planar de 2 eslabones.
 *  x = adelante (mm, + = hacia delante)     y = abajo (mm, + = hacia el suelo)
 *  Devuelve th1 (femur respecto a la vertical) y th2 (angulo relativo rodilla).
 */
void legIK(float x, float y, float &th1, float &th2) {
  float r    = sqrtf(x * x + y * y);
  float rmax = (L1 + L2) * 0.995f;
  float rmin = fabsf(L1 - L2) + 0.5f;

  if (r > rmax)      { float k = rmax / r; x *= k; y *= k; r = rmax; }
  else if (r < rmin) { float k = rmin / r; x *= k; y *= k; r = rmin; }

  float c2 = (r * r - L1 * L1 - L2 * L2) / (2.0f * L1 * L2);
  c2 = constrain(c2, -1.0f, 1.0f);

  th2 = (float)KNEE_DIR * acosf(c2);
  th1 = atan2f(x, y) - atan2f(L2 * sinf(th2), L1 + L2 * cosf(th2));
}

void computeNeutral() {
  for (uint8_t i = 0; i < NUM_LEGS; i++)
    legIK(legs[i].xOffset, BODY_H, th1_0[i], th2_0[i]);
}

/*  Coloca el pie de la pata i en (x,y) mm respecto al eje de cadera. */
void applyLeg(uint8_t i, float x, float y) {
  float th1, th2;
  legIK(x, y, th1, th2);

  float hipAng  = legs[i].hipNeutral  +
                  legs[i].hipSign  * (th1 - th1_0[i]) * RAD2DEG;
  float kneeAng = legs[i].kneeNeutral +
                  legs[i].kneeSign * (th2 - th2_0[i]) * RAD2DEG;

  servoWrite(legs[i].hipCh,  hipAng);
  servoWrite(legs[i].kneeCh, kneeAng);
}

/*  Trayectoria del pie para la fase p (0..1) de la pata i.
 *  p=0..0.25  vuelo : sube y va de atras a adelante
 *  p=0.25..1  apoyo : pie en el suelo, retrocede a velocidad constante
 */
void gaitPose(uint8_t i, float p, float amp, float &x, float &y) {
  float half = STEP_LEN * 0.5f * amp;

  if (p < SWING_FRAC) {
    float s = p / SWING_FRAC;
    float e = 0.5f - 0.5f * cosf(PI * s);
    x = -half + 2.0f * half * e;
    y = BODY_H - STEP_H * amp * sinf(PI * s);
  } else {
    float s = (p - SWING_FRAC) / (1.0f - SWING_FRAC);
    x = half - 2.0f * half * s;
    y = BODY_H;
  }
  x += legs[i].xOffset;
}

/* ---------------------------- modos -------------------------------------- */

void standPose() {
  for (uint8_t i = 0; i < NUM_LEGS; i++) {
    servoWrite(legs[i].hipCh,  legs[i].hipNeutral);
    servoWrite(legs[i].kneeCh, legs[i].kneeNeutral);
  }
}

void doWalk(unsigned long now) {
  float amp = 1.0f;
  if (now - t0 < RAMP_MS) amp = (float)(now - t0) / (float)RAMP_MS;

  float cp = fmodf((float)(now - t0) / (float)CYCLE_MS, 1.0f);

  for (uint8_t i = 0; i < NUM_LEGS; i++) {
    
    float p = cp + (float)i / (float)NUM_LEGS;
    if (p >= 1.0f) p -= 1.0f;

    float x, y;
    gaitPose(i, p, amp, x, y);
    applyLeg(i, x, y);
  }
}

void doTest(unsigned long now) {
  unsigned long dur = 3000;
  float p = (float)(now - testT0) / (float)dur;

  if (p >= 1.0f) { mode = M_STAND; standPose(); Serial.println("test fin"); return; }

  for (uint8_t i = 0; i < NUM_LEGS; i++) {
    if (i == testLeg) {
      float x, y;
      gaitPose(i, p, 1.0f, x, y);
      applyLeg(i, x, y);
    } else {
      applyLeg(i, legs[i].xOffset, BODY_H);
    }
  }
}

/* --------------------------- consola serie ------------------------------- */

void printHelp() {
  Serial.println();
  Serial.println("  ?                     esta ayuda");
  Serial.println("  g                     caminar");
  Serial.println("  s                     parar / de pie");
  Serial.println("  t <pata>              probar una pata (0..3)");
  Serial.println("  m <canal> <grados>    mover un servo (0..15)");
  Serial.println("  i <pata> <x> <y>      pie a posicion mm (+x adelante, +y abajo)");
  Serial.println("  f <pata> h|k          invertir signo hombro/rodilla");
  Serial.println("  x <pata> <mm>         adelantar(+)/atrasar(-) el pie de la pata");
  Serial.println("  geo <L1> <L2> <alto>  femur, tibia y altura de cadera (mm)");
  Serial.println("  p                     ver configuracion");
  Serial.println("  d                     volcar config como codigo C++");
  Serial.println("  patas: 0=FL 1=RR 2=FR 3=RL");
  Serial.println();
}

void printConfig() {
  Serial.printf("L1=%.1f  L2=%.1f  BODY_H=%.1f  KNEE_DIR=%d\n", L1, L2, BODY_H, KNEE_DIR);
  Serial.printf("STEP_LEN=%.1f  STEP_H=%.1f  CYCLE_MS=%lu\n", STEP_LEN, STEP_H, (unsigned long)CYCLE_MS);
  for (uint8_t i = 0; i < NUM_LEGS; i++) {
    Serial.printf("  %u %s hip=%u sig=%+d neu=%.1f | knee=%u sig=%+d neu=%.1f | xoff=%.1f\n",
                  i, legs[i].name,
                  legs[i].hipCh,  legs[i].hipSign,  legs[i].hipNeutral,
                  legs[i].kneeCh, legs[i].kneeSign, legs[i].kneeNeutral,
                  legs[i].xOffset);
  }
}

void dumpConfig() {
  Serial.println("/* pega esto en el array legs[] */");
  for (uint8_t i = 0; i < NUM_LEGS; i++) {
    Serial.printf("  { %u, %u, %+d, %+d, %.1ff, %.1ff, %.1ff, \"%s\" },\n",
                  legs[i].hipCh, legs[i].kneeCh,
                  legs[i].hipSign, legs[i].kneeSign,
                  legs[i].hipNeutral, legs[i].kneeNeutral,
                  legs[i].xOffset, legs[i].name);
  }
  Serial.printf("// L1=%.1ff  L2=%.1ff  BODY_H=%.1ff  KNEE_DIR=%d\n", L1, L2, BODY_H, KNEE_DIR);
}

void handleLine(char *line) {
  char cmd[8] = {0};
  if (sscanf(line, "%7s", cmd) != 1) return;

  if (!strcmp(cmd, "?")) { printHelp(); return; }

  if (!strcmp(cmd, "g")) {
    computeNeutral();
    t0 = millis();
    mode = M_WALK;
    Serial.println(">> WALK");
    return;
  }
  if (!strcmp(cmd, "s")) {
    mode = M_STAND; standPose();
    Serial.println(">> STAND");
    return;
  }
  if (!strcmp(cmd, "p")) { printConfig(); return; }
  if (!strcmp(cmd, "d")) { dumpConfig(); return; }

  if (!strcmp(cmd, "t")) {
    int l = -1;
    if (sscanf(line, "t %d", &l) == 1 && l >= 0 && l < 4) {
      testLeg = (uint8_t)l; testT0 = millis(); mode = M_TEST;
      Serial.printf(">> test pata %d (%s)\n", l, legs[l].name);
    } else Serial.println("uso: t <0..3>");
    return;
  }

  if (!strcmp(cmd, "m")) {
    int ch; float a;
    if (sscanf(line, "m %d %f", &ch, &a) == 2 && ch >= 0 && ch < 16) {
      mode = M_MANUAL;
      appliedAngle[ch] = -100.0f;      // fuerza el envio
      servoWrite((uint8_t)ch, a);
      Serial.printf(">> ch%d = %.1f\n", ch, a);
    } else Serial.println("uso: m <0..15> <0..180>");
    return;
  }

  if (!strcmp(cmd, "i")) {
    int l; float x, y;
    if (sscanf(line, "i %d %f %f", &l, &x, &y) == 3 && l >= 0 && l < 4) {
      mode = M_MANUAL;
      for (uint8_t k = 0; k < NUM_LEGS; k++) {
        if (k == (uint8_t)l) applyLeg(k, x, y);
        else                 applyLeg(k, legs[k].xOffset, BODY_H);
      }
      Serial.printf(">> pata %d pie en (%.0f, %.0f)\n", l, x, y);
    } else Serial.println("uso: i <0..3> <x_mm> <y_mm>");
    return;
  }

  if (!strcmp(cmd, "f")) {
    int l; char j;
    if (sscanf(line, "f %d %c", &l, &j) == 2 && l >= 0 && l < 4) {
      if (j == 'h') { legs[l].hipSign  = -legs[l].hipSign;  Serial.printf(">> %s hipSign = %+d\n",  legs[l].name, legs[l].hipSign);  }
      else if (j == 'k') { legs[l].kneeSign = -legs[l].kneeSign; Serial.printf(">> %s kneeSign = %+d\n", legs[l].name, legs[l].kneeSign); }
      else Serial.println("usa h (hombro) o k (rodilla)");
      computeNeutral();
      standPose();
    } else Serial.println("uso: f <0..3> h|k");
    return;
  }

  if (!strcmp(cmd, "x")) {
    int l; float v;
    if (sscanf(line, "x %d %f", &l, &v) == 2 && l >= 0 && l < 4) {
      legs[l].xOffset = v;
      computeNeutral();
      mode = M_STAND; standPose();
      Serial.printf(">> %s xOffset = %.1f mm\n", legs[l].name, v);
    } else Serial.println("uso: x <0..3> <mm>");
    return;
  }

  if (!strcmp(cmd, "geo")) {
    float a, b, c;
    if (sscanf(line, "geo %f %f %f", &a, &b, &c) == 3) {
      L1 = a; L2 = b; BODY_H = c;
      computeNeutral();
      mode = M_STAND; standPose();
      Serial.printf(">> L1=%.1f L2=%.1f BODY_H=%.1f\n", L1, L2, BODY_H);
    } else Serial.println("uso: geo <L1> <L2> <alto>");
    return;
  }

  Serial.printf("comando desconocido: %s  (escribe ? para ayuda)\n", cmd);
}

void readSerial() {
  static char line[64];
  static uint8_t n = 0;

  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (n) { line[n] = 0; handleLine(line); n = 0; }
    } else if (n < sizeof(line) - 1) {
      line[n++] = c;
    }
  }
}

/* ------------------------------ setup ------------------------------------ */

void setup() {
  Serial.begin(115200);
  delay(400);

  Serial.println();
  Serial.println("=== CUADRUPEDO 8 SERVOS | ESP32 + PCA9685 | IK 2 eslabones ===");

  Wire.begin(PIN_SDA, PIN_SCL);
  pwm.begin();
  pwm.setPWMFreq(50);
  delay(10);

  for (uint8_t ch = 0; ch < 16; ch++) appliedAngle[ch] = -100.0f;

  computeNeutral();
  standPose();

  printHelp();
  printConfig();

  Serial.println("Pose neutra. Esperando 1.5 s y arrancando la marcha...");
  delay(1500);

  computeNeutral();
  t0 = millis();
  mode = M_WALK;
  Serial.println(">> WALK");
}

/* ------------------------------- loop ------------------------------------ */

void loop() {
  readSerial();
  unsigned long now = millis();

  switch (mode) {
    case M_WALK:   doWalk(now);   break;
    case M_TEST:   doTest(now);   break;
    case M_STAND:  /* la pose ya esta aplicada */ break;
    case M_MANUAL: /* manda la consola */         break;
  }

  delay(8);
}
