@echo off
title QuadSim Setup - Tien trinh cai dat
color 0B

echo ===================================================
echo        CAI DAT MOI TRUONG QUADSIM (WINDOWS)
echo ===================================================
echo LUU Y: KHONG CLICK CHUOT VAO MAN HINH TRONG LUC CHAY!
echo ===================================================
echo.

:: Kiem tra Python
echo [##----------------] 10%% - Kiem tra Python...
python --version >nul 2>&1
if errorlevel 1 (
    echo [LOI] Khong tim thay Python. Vui long cai Python 3.10+ va them vao PATH.
    pause
    exit /b
)

:: Kiem tra va tao moi truong ao
if exist .venv (
    echo [####---------------] 20%% - Phat hien moi truong ao cu.
    echo Dang kiem tra tinh trang...
    call .venv\Scripts\activate.bat
    python -m pip --version >nul 2>&1
    if errorlevel 1 (
        echo [!] Moi truong ao cu bi loi (thieu pip).
        echo =^> Dang xoa de tao lai...
        deactivate 2>nul
        rmdir /s /q .venv
        echo [#####--------------] 25%% - Tao moi truong ao moi...
        python -m venv .venv
        if errorlevel 1 (
            echo [LOI] Khong the tao moi truong ao. Kiem tra quyen thu muc.
            pause
            exit /b
        )
    ) else (
        echo [######-------------] 30%% - Moi truong ao hop le, tiep tuc...
    )
) else (
    echo [#####--------------] 25%% - Tao moi truong ao moi...
    python -m venv .venv
    if errorlevel 1 (
        echo [LOI] Khong the tao moi truong ao. Kiem tra quyen thu muc.
        pause
        exit /b
    )
)

:: Kich hoat moi truong ao
echo [#########----------] 45%% - Kich hoat moi truong ao...
call .venv\Scripts\activate.bat
if errorlevel 1 (
    echo [LOI] Khong the kich hoat moi truong ao.
    pause
    exit /b
)

:: Cap nhat pip
echo [##########--------] 50%% - Cap nhat pip...
python -m pip install --upgrade pip -q

echo.
echo ===================================================
echo               MENU CAI DAT THU VIEN
echo ===================================================
echo 1. Phien ban binh thuong (CLI - chay tren Terminal)
echo 2. Phien ban co Giao dien Web (GUI - Streamlit)
echo 3. Phien ban Giao dien + AI khoanh vung (GUI + SAM)
echo ===================================================
set /p choice="Nhap lua chon cua ban (1, 2 hoac 3): "

if "%choice%"=="1" goto opt1
if "%choice%"=="2" goto opt2
if "%choice%"=="3" goto opt3
goto invalid

:opt1
echo.
echo [=============-------] 65%% - Dang tai thu vien co ban...
echo ETA: ~1 phut (Vui long cho, dung an phim gi)...
python -m pip install -r requirements.txt
if errorlevel 1 (
    echo [LOI] Cai dat that bai. Kiem tra ket noi mang hoac file requirements.txt.
    pause
    exit /b
)
goto finish

:opt2
echo.
echo [=============-------] 65%% - Dang tai thu vien GUI...
echo ETA: ~1-2 phut (Vui long cho, dung an phim gi)...
python -m pip install -r requirements_gui.txt
if errorlevel 1 (
    echo [LOI] Cai dat that bai. Kiem tra ket noi mang hoac file requirements_gui.txt.
    pause
    exit /b
)
goto finish

:opt3
echo.
echo [=============-------] 65%% - Dang tai thu vien GUI co ban...
python -m pip install -r requirements_gui.txt
if errorlevel 1 (
    echo [LOI] Cai dat that bai. Kiem tra ket noi mang hoac file requirements_gui.txt.
    pause
    exit /b
)
echo.
echo [==============------] 75%% - Dang tai thu vien AI (PyTorch/SAM)...
echo ETA: ~3-6 phut. PHAN NAY RAT NANG, VUI LONG CHO...
python -m pip install -r requirements_gui_sam.txt
if errorlevel 1 (
    echo [LOI] Cai dat that bai. Kiem tra ket noi mang hoac file requirements_gui_sam.txt.
    pause
    exit /b
)
echo.
echo [===================--] 90%% - Dang cai MobileSAM...
python -m pip install "git+https://github.com/ChaoningZhang/MobileSAM.git"
if errorlevel 1 (
    echo [LOI] Cai dat MobileSAM that bai. Ban co the thu cai thu cong sau.
    echo Tiep tuc...
)
goto finish

:invalid
echo.
echo [!] Lua chon khong hop le. Vui long chon 1, 2 hoac 3.
pause
exit /b

:finish
echo.
echo [====================] 100%% - CAI DAT THANH CONG!
echo ===================================================
echo De chay chuong trinh, hay go lenh sau vao Terminal:
if "%choice%"=="1" (
    echo =^> python main.py
) else if "%choice%"=="2" (
    echo =^> streamlit run app_gui.py
) else if "%choice%"=="3" (
    echo =^> streamlit run app_gui.py
)
echo.
echo Luu y: Neu chon GUI, lan dau chay co the can cai them thu vien
echo         neu Streamlit yeu cau.
echo ===================================================
pause