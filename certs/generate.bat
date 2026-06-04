@echo off
setlocal

set SCRIPT_DIR=%~dp0
set CERT_PEM=%SCRIPT_DIR%bridge_cert.pem
set KEY_PEM=%SCRIPT_DIR%bridge_key.pem
set CFG_FILE=%SCRIPT_DIR%openssl.cnf
set HEADER_OUT=%SCRIPT_DIR%..\src\ssl_cert.h
set PY_SCRIPT=%SCRIPT_DIR%make_header.py

where openssl >nul 2>nul
if errorlevel 1 (
  echo [ERROR] OpenSSL not found in PATH.
  echo Install OpenSSL and ensure openssl.exe is available.
  exit /b 1
)

echo Generating OpenSSL config...
(
  echo [req]
  echo prompt = no
  echo default_md = sha256
  echo distinguished_name = dn
  echo x509_extensions = v3_ca
  echo(
  echo [dn]
  echo C = CH
  echo ST = VD
  echo L = Lausanne
  echo O = GoPro-Bridge
  echo CN = GoPro-Bridge
  echo(
  echo [v3_ca]
  echo subjectAltName = @alt_names
  echo basicConstraints = critical,CA:TRUE
  echo keyUsage = critical,digitalSignature,keyEncipherment,keyCertSign
  echo extendedKeyUsage = serverAuth
  echo(
  echo [alt_names]
  echo IP.1 = 192.168.4.1
) > "%CFG_FILE%"

echo Generating EC key...
openssl ecparam -name prime256v1 -genkey -noout -out "%KEY_PEM%"
if errorlevel 1 exit /b 1

echo Generating self-signed cert...
openssl req -x509 -new -key "%KEY_PEM%" -days 3650 -sha256 -out "%CERT_PEM%" -config "%CFG_FILE%" -extensions v3_ca
if errorlevel 1 exit /b 1

echo Generating C header...
python "%PY_SCRIPT%" "%CERT_PEM%" "%KEY_PEM%" "%HEADER_OUT%"
if errorlevel 1 exit /b 1

echo Done.
echo Generated:
echo   %CERT_PEM%
echo   %KEY_PEM%
echo   %HEADER_OUT%
