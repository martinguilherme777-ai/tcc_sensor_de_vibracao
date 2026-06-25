[README.md](https://github.com/user-attachments/files/29318756/README.md)
# ESP32 + LIS3DH: coleta de vibracao por ESP-NOW

Esta pasta contem uma versao pronta para GitHub dos codigos de coleta e geracao de figuras.

## Arquivos

- `master_lis3dh_2000hz/master_lis3dh_2000hz.ino`: ESP32 com LIS3DH. Coleta 10 s a 2000 Hz.
- `receptor_espnow_serial/receptor_espnow_serial.ino`: ESP32 receptor. Encaminha os pacotes ao computador pela Serial.
- `salvar_coleta_10s.py`: recebe os pacotes, reconstrui a coleta e salva CSV/XLSX.
- `gerar_figuras_tcc_enfoque.py`: gera graficos de tempo, FFT 0-400 Hz e FFT 0-1000 Hz.

## Configuracao rapida

1. Ajuste, se necessario, os pinos I2C no master:
   - `SDA_PIN`
   - `SCL_PIN`

2. Ajuste a escala do acelerometro no master:
   - `ACCEL_RANGE_G 2`, `4`, `8` ou `16`

3. Use a mesma escala no Python:

```bash
python salvar_coleta_10s.py --escala-g 8
```

4. Se a porta serial nao for detectada automaticamente:

```bash
python salvar_coleta_10s.py --listar-portas
python salvar_coleta_10s.py --porta COM5 --escala-g 8
```

No Linux, a porta pode ser algo como `/dev/ttyUSB0`.

5. Gere as figuras:

```bash
python gerar_figuras_tcc_enfoque.py --pasta coletas --saida figuras --escala-g 8
```

## Dependencias Python

```bash
pip install pyserial pandas numpy matplotlib scipy
```

## Observacoes

- Os CSVs gerados usam as colunas `tempo,x,y,z`.
- As aceleracoes sao salvas em `g`.
- Os graficos de FFT exibem amplitude em `m/s2`.
- O firmware usa o LIS3DH em modo low-power, com 8 bits efetivos.
- Por padrao, a coleta tem 10 s, 2000 Hz e escala `+/-8 g`.
