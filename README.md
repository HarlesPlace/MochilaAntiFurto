# Mochila Antifurto com ESP32
[![MIT License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

Sistema embarcado para monitoramento de movimentação via acelerômetro MPU6050 e rastreamento por GPS NEO-6M, com comunicação via GSM.

---

## Funcionalidades

- Leitura contínua dos dados do acelerômetro MPU6050 via I2C
- Detecção de movimento com lógica de filtro temporal para evitar falsos alarmes
- Integração planejada com GPS NEO-6M para rastreamento da mochila
- Envio de alertas via módulo GSM

---

## Hardware Utilizado

- ESP32 DevKit v1 (38 pinos)
- Sensor acelerômetro e giroscópio MPU6050 conectado via I2C (GPIO 21 - SDA, GPIO 22 - SCL)
- Módulo GPS NEO-6M conectado via UART2 (GPIO 16 - TX, GPIO 17 - RX)
- (Opcional) Módulo GSM via UART1

---

## Conexões

| Componente | Sinal    | Pino ESP32 |
|------------|----------|------------|
| MPU6050    | SDA      | GPIO 21    |
| MPU6050    | SCL      | GPIO 22    |
| GPS NEO-6M | TX       | GPIO 16    |
| GPS NEO-6M | RX       | GPIO 17    |
| GSM        | TX       | UART1 TX   |
| GSM        | RX       | UART1 RX   |

---

## Como Compilar e Rodar

1. Instale o ESP-IDF (versão 5.0 ou superior).  
2. Clone este repositório:  
```bash
git clone https://github.com/HarlesPlace/MochilaAntiFurto
cd mochila-antifurto
```
3. Configure o alvo para ESP32:
```bash
Copiar
Editar
idf.py set-target esp32
```
4. Compile e grave:

```bash
Copiar
Editar
idf.py build
idf.py flash
idf.py monitor
```
5. Conecte os sensores e aguarde as leituras no monitor serial.

---

## Licença
Este projeto está licenciado sob a licença MIT.
Veja o arquivo [LICENSE](LICENSE) para mais detalhes.

A biblioteca [minmea](https://github.com/kosma/minmea) utilizada neste projeto também está licenciada sob a licença MIT.

---

## Créditos
Desenvolvido por:
- Edmond Léandre Émile Jean
- Isabela Mosna Esteves
- Larissa de Souza Fontes
- [Nicolas Sobral Cruz](https://github.com/HarlesPlace)

Este trabalho foi realizado como parte da disciplina de Sistemas Embarcados do curso de Engenharia Mecatrônica
da Escola Politécnica da USP.

---

## Contato
Para dúvidas ou sugestões, abra uma issue ou envie um e-mail para [nicolasunif@gmail.com].

Obrigado por visitar nosso projeto!