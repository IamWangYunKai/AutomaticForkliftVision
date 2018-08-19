#ifndef _EXIT_H_
#define _EXIT_H_

#include <robokit/core/logger.h>
#include <robokit/core/meminfo.h>

#ifdef RBK_SYS_WINDOWS
#else
#include <signal.h>
#endif

static void ExitRBK() {
    rbk::ParamsServer::Instance()->closeDB();
    rbk::core::sys::MemoryInfo memInfo;
    memInfo.freeSystemMem();
    google::protobuf::ShutdownProtobufLibrary();
    LogInfo(formatLogText("Used system memory : " << memInfo.usedSystemMem() / 1024 / 1024 << " GB"));
    LogInfo(formatLogText("Free system memory : " << memInfo.freeSystemMem() / 1024 / 1024 << " GB"));
    LogInfo(formatLogText("Robokit physical memory usage : " << memInfo.usedPhysicalMem() / 1024 << " MB"));
    LogInfo(formatLogText("Robokit virtual memory usage  : " << memInfo.usedVirtualMem() / 1024 << " MB"));
    LogInfo(formatLogText("Robokit Max physical memory usage : " << memInfo.usedPhysicalMemMax() / 1024 << " MB"));
    LogInfo(formatLogText("Robokit Max virtual memory usage  : " << memInfo.usedVirtualMemMax() / 1024 << " MB"));
    rbk::Logger::destroy();
#ifdef RBK_SYS_WINDOWS
    ExitProcess(0);
#else
    _exit(0);
#endif
}

#ifdef RBK_SYS_WINDOWS
static BOOL WINAPI ConsoleHandler(DWORD ev) {
    switch (ev) {
    case CTRL_C_EVENT:
        LogInfo(formatLogText("ctrl+c received, exit process..."));
        break;
    case CTRL_CLOSE_EVENT:
        LogInfo(formatLogText("close received, exit process..."));
        break;
    case CTRL_LOGOFF_EVENT:
        LogInfo(formatLogText("user is logging off, exit process..."));
        break;
    case CTRL_SHUTDOWN_EVENT:
        LogInfo(formatLogText("shutdown! exit process..."));
        break;
    default:
        break;
    }
    ExitRBK();
    return TRUE;
}

LRESULT CALLBACK MainWndProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam) {
    switch (msg) {
    case WM_QUERYENDSESSION:
        LogInfo(formatLogText("Received WM_QUERYENDSESSION, need shutdown!"));
        return TRUE;

    case WM_ENDSESSION:
        LogInfo(formatLogText("Received WM_ENDSESSION, shutdown!"));
        ExitRBK();
        return TRUE;

    case WM_DESTROY:
    {
        LogInfo(formatLogText("Received WM_DESTROY"));
        PostQuitMessage(0);
    }
    break;

    case WM_CLOSE:
        LogInfo(formatLogText("Received WM_CLOSE"));
        break;
    default:
        return DefWindowProc(hwnd, msg, wParam, lParam);
        break;
    }
    return TRUE;
}

void CreateInvisibleWindow() {
    HWND hwnd;
    WNDCLASS wc = { 0 };
    wc.lpfnWndProc = (WNDPROC)MainWndProc;
    wc.hInstance = GetModuleHandle(NULL);
    wc.hIcon = LoadIcon(GetModuleHandle(NULL), L"RBKWindow");
    wc.lpszClassName = L"RBKWindow";
    RegisterClass(&wc);

    hwnd = CreateWindowEx(0, L"RBKWindow", L"RBKWindow", WS_OVERLAPPEDWINDOW, CW_USEDEFAULT, CW_USEDEFAULT, CW_USEDEFAULT, CW_USEDEFAULT, (HWND)NULL, (HMENU)NULL, GetModuleHandle(NULL), (LPVOID)NULL);
    if (!hwnd) {
        printf("FAILED to create window!!! %d\n", GetLastError());
    }
}

DWORD WINAPI RunInvisibleWindowThread(LPVOID lpParam) {
    MSG msg;
    CreateInvisibleWindow();
    while (GetMessage(&msg, (HWND)NULL, 0, 0))
    {
        TranslateMessage(&msg);
        DispatchMessage(&msg);
    }
    return 0;
}

#else
static void sig_handler(int sig) {
    if (sig == SIGINT) {
#ifdef RBK_SYS_LINUX
        LogInfo(formatLogText("ctrl+c received, exit process..."));
#else
        LogInfo(formatLogText("control+c received, exit process..."));
#endif
        ExitRBK();
    }
}
#endif

// TODO
static void lockFile(FILE* fp) {
#ifdef RBK_SYS_WINDOWS

    OVERLAPPED overlapped;
    memset(&overlapped, 0, sizeof(overlapped));

    LockFileEx(fp,LOCKFILE_EXCLUSIVE_LOCK | LOCKFILE_FAIL_IMMEDIATELY,0,MAXDWORD,MAXDWORD,&overlapped);
#endif
}

#endif // ~_EXIT_H_
