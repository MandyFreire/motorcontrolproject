// Leis de controle em C puro – porte linha a linha de motorlib/controllers.py.
// Não depende da API Arduino: tests/test_paridade_c.py compila este arquivo no PC
// e compara a saída com a implementação Python, amostra a amostra.
#pragma once
#include <math.h>
#include "parametros_gerados.h"

// Divisão float no AVR custa ~500 ciclos; dividir por constante vira multiplicação pelo inverso.
#define TS_INV (1.0f / TS_S)

static inline float clampf(float x, float lo, float hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}

// ---------------------------------------------------------------- PID
// Derivada sobre a medição com filtro de 1a ordem; anti-windup por integração condicional.
typedef struct {
  float integ, d, y_prev;
  int iniciado;
} EstadoPID;

static inline void pid_reset(EstadoPID *s) { s->integ = 0; s->d = 0; s->y_prev = 0; s->iniciado = 0; }

static inline float pid_update(EstadoPID *s, float r, float y) {
  const float alpha = PID_TF / (PID_TF + TS_S);
  float e = r - y;
  if (!s->iniciado) { s->y_prev = y; s->iniciado = 1; }
  s->d = alpha * s->d + (1.0f - alpha) * (y - s->y_prev) * TS_INV;
  s->y_prev = y;
  float integ_new = s->integ + e * TS_S;
  float u_raw = PID_KP * e + PID_KI * integ_new - PID_KD * s->d;
  if ((u_raw > V_MAX && e > 0) || (u_raw < -V_MAX && e < 0)) {
    u_raw = PID_KP * e + PID_KI * s->integ - PID_KD * s->d;   // congela o integrador
  } else {
    s->integ = integ_new;
  }
  return clampf(u_raw, -V_MAX, V_MAX);
}

// ---------------------------------------------------------------- Fuzzy PI incremental
typedef struct {
  float u, e_prev;
  int iniciado;
} EstadoFuzzy;

static inline void fuzzy_reset(EstadoFuzzy *s) { s->u = 0; s->e_prev = 0; s->iniciado = 0; }

static inline float fz_tab(int i, int j) { return pgm_read_float(&FZ_TABELA[i][j]); }

static inline float fuzzy_superficie(float en, float den) {
  float fx = (clampf(en, -1.0f, 1.0f) + 1.0f) * 0.5f * (FZ_N - 1);
  float fy = (clampf(den, -1.0f, 1.0f) + 1.0f) * 0.5f * (FZ_N - 1);
  int i = (int)fx; if (i > FZ_N - 2) i = FZ_N - 2;
  int j = (int)fy; if (j > FZ_N - 2) j = FZ_N - 2;
  float tx = fx - i, ty = fy - j;
  return (1 - tx) * (1 - ty) * fz_tab(i, j) + tx * (1 - ty) * fz_tab(i + 1, j)
       + (1 - tx) * ty * fz_tab(i, j + 1) + tx * ty * fz_tab(i + 1, j + 1);
}

static inline float fuzzy_update(EstadoFuzzy *s, float r, float y) {
  float e = r - y;
  if (!s->iniciado) { s->e_prev = e; s->iniciado = 1; }
  float du = FZ_KU * fuzzy_superficie(FZ_KE * e, FZ_KDE * (e - s->e_prev));
  s->e_prev = e;
  s->u = clampf(s->u + du, -V_MAX, V_MAX);
  return s->u;
}

// ---------------------------------------------------------------- MLP (clone do PID)
// Entradas [e, integral(e), derivada filtrada de y] -> 8 ReLU -> 1. Integrador e filtro
// ficam fora da rede, com o mesmo anti-windup do PID.
typedef struct {
  float integ, d, y_prev;
  int iniciado;
} EstadoMLP;

static inline void mlp_reset(EstadoMLP *s) { s->integ = 0; s->d = 0; s->y_prev = 0; s->iniciado = 0; }

static inline float mlp_forward(float x0, float x1, float x2) {
  float x[MLP_NIN] = {x0, x1, x2};
  for (int k = 0; k < MLP_NIN; k++) x[k] = (x[k] - MLP_XMEAN[k]) * MLP_XSTD_INV[k];
  float out = MLP_B2;
  for (int h = 0; h < MLP_NH; h++) {
    float a = MLP_B1[h];
    for (int k = 0; k < MLP_NIN; k++) a += x[k] * MLP_W1[k][h];
    if (a > 0) out += a * MLP_W2[h];
  }
  return out * MLP_YSTD + MLP_YMEAN;
}

static inline float mlp_update(EstadoMLP *s, float r, float y) {
  const float alpha = PID_TF / (PID_TF + TS_S);
  float e = r - y;
  if (!s->iniciado) { s->y_prev = y; s->iniciado = 1; }
  s->d = alpha * s->d + (1.0f - alpha) * (y - s->y_prev) * TS_INV;
  s->y_prev = y;
  float integ_new = s->integ + e * TS_S;
  float u_raw = mlp_forward(e, integ_new, s->d);
  if ((u_raw > V_MAX && e > 0) || (u_raw < -V_MAX && e < 0)) {
    u_raw = mlp_forward(e, s->integ, s->d);
  } else {
    s->integ = integ_new;
  }
  return clampf(u_raw, -V_MAX, V_MAX);
}
