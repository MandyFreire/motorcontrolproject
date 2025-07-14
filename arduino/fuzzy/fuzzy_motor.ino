#include <Fuzzy.h>

#define PWM 9

Fuzzy *fuzzy = new Fuzzy();

void setup() {
  pinMode(PWM, OUTPUT);
  Serial.begin(9600);

  FuzzyInput *erro = new FuzzyInput(1);
  FuzzySet *negativo = new FuzzySet(-10, -10, 0, 0);
  erro->addFuzzySet(negativo);

  FuzzyOutput *saida = new FuzzyOutput(1);
  FuzzySet *reduzir = new FuzzySet(0, 0, 100, 150);
  saida->addFuzzySet(reduzir);

  fuzzy->addFuzzyInput(erro);
  fuzzy->addFuzzyOutput(saida);

  FuzzyRuleAntecedent *ifErroNegativo = new FuzzyRuleAntecedent();
  ifErroNegativo->joinSingle(negativo);

  FuzzyRuleConsequent *thenReduzir = new FuzzyRuleConsequent();
  thenReduzir->addOutput(reduzir);

  FuzzyRule *rule = new FuzzyRule(1, ifErroNegativo, thenReduzir);
  fuzzy->addFuzzyRule(rule);
}

void loop() {
  float rpm = analogRead(A0) / 10.0;
  float erro = 100.0 - rpm;

  fuzzy->setInput(1, erro);
  fuzzy->fuzzify();
  float output = fuzzy->defuzzify(1);

  analogWrite(PWM, output);
  Serial.println(output);
}
