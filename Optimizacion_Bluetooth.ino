#include <SoftwareSerial.h>
#include <math.h>

SoftwareSerial BTSerial(10, 11); // RX, TX

const int N = 64;          // Número de muestras
float signal[N];            // Señal simulada
float spectrum_float[N];    // Espectro en punto flotante
int spectrum_fixed[N];      // Espectro en punto fijo (optimizado)

void setup() {
  Serial.begin(9600);
  BTSerial.begin(9600);

  // Generar señal discreta simulada (ejemplo: seno compuesto)
  for (int n = 0; n < N; n++) {
    signal[n] = sin(2 * PI * n / N) + 0.5 * sin(4 * PI * n / N);
  }
}

void loop() {
  unsigned long t1, t2;

  // --- Procesamiento en punto flotante ---
  t1 = micros();
  for (int k = 0; k < N; k++) {
    float real = 0, imag = 0;
    for (int n = 0; n < N; n++) {
      float angle = -2 * PI * k * n / N;
      real += signal[n] * cos(angle);
      imag += signal[n] * sin(angle);
    }
    spectrum_float[k] = sqrt(real * real + imag * imag);
  }
  t2 = micros();
  unsigned long tiempo_float = t2 - t1;

  // --- Procesamiento optimizado en punto fijo ---
  t1 = micros();
  for (int k = 0; k < N; k++) {
    long real = 0, imag = 0;
    for (int n = 0; n < N; n++) {
      int angleIndex = (k * n) % N;
      int cosVal = (int)(1000 * cos(-2 * PI * angleIndex / N));
      int sinVal = (int)(1000 * sin(-2 * PI * angleIndex / N));
      real += (int)(signal[n] * 1000) * cosVal / 1000;
      imag += (int)(signal[n] * 1000) * sinVal / 1000;
    }
    spectrum_fixed[k] = sqrt(real * real + imag * imag) / 1000;
  }
  t2 = micros();
  unsigned long tiempo_fixed = t2 - t1;

  // --- Envío de datos al Serial Plotter ---
  // Se envían dos series: tiempo_float y tiempo_fixed
  Serial.print(tiempo_float);
  Serial.print("\t");          // separador para graficar dos curvas
  Serial.println(tiempo_fixed);

  // --- También se envían al HC-05 ---
  BTSerial.print("Tiempo flotante: ");
  BTSerial.print(tiempo_float);
  BTSerial.println(" us");

  BTSerial.print("Tiempo fijo: ");
  BTSerial.print(tiempo_fixed);
  BTSerial.println(" us");

  delay(1000); // Espera entre iteraciones
}