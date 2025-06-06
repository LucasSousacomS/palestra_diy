﻿# Projeto de demonstração - Semana da Engenharia

Este projeto tem como objetivo demonstrar o funcionamento de um projeto no estilo DIY, envolvendo o uso de micrcontroladores, sensores, atuadores e impressão 3D.

## Materiais utilizados:

- 4 resistores de 100 ohm
- 1 LED difuso verde
- 1 LED difuso amarelo
- 1 LED difuso vermelho
- 1 Arduino Uno
- 1 sensor TOF VL53L0X
- 1 botão sem trava 12mm
- Cabos flexíveis vermelho, azul e amarelo 24 AWG

## Pinagem

LED verde: pino 2
LED amarelo: pino 3
LED vermelho: pino 4
Botão: pino 5

### Alimentação e comunicação do sensor VL53L0X:
VIN = 3,3V
GND = GND
SDA = SDA
SCL = SCL

## Funcionamento

Este projeto possui dois modos de funcionamento: modo semáforo e modo distância. Ao inicializar o Arduino, o modo semáforo estará acionado. Nesse modo, o LED vermelho estará acionado inicialmente. Ao pressionar o botão, após 2 segundos, o LED verde será acionado. Após 10 segundos, o LED verde será desligado e o amarelo acionado por 2 seegundos. Após os 2 segundos, o LED vermelho será acionado novamente, reiniciando o processo. Caso o usuário segure o botão por 2 segundos, os 3 LEDs irão piscar em conjunto, indicando que o modo será alterado. No modo distância, ao aproximar algum objeto do sensor, os LEDs serão acionados na seguinte ordem?

Distância menor do que 180 mm: LED verde
Distância menor do que 120 mm: LEDs amarelo e verde
Distância menor do que 80 mm: LEDs vermelho, amarelo e verde

## Resistores calculados para os LEDs:

Resistor para o LED vermelho = 160 ohm

Resistor para o LED amarelo = 135 ohm

Resistor para o LED verde = 75

## Resistores utilizados

Resistor para o LED vermelho = 100 ohm

Resistor para o LED amarelo = 100 ohm

Resistor para o LED verde = 77 ohm (100 em paralelo com 330)
