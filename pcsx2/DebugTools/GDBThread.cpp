/*  PCSX2 - PS2 Emulator for PCs
*  Copyright (C) 2002-2010  PCSX2 Dev Team
*
*  PCSX2 is free software: you can redistribute it and/or modify it under the terms
*  of the GNU Lesser General Public License as published by the Free Software Found-
*  ation, either version 3 of the License, or (at your option) any later version.
*
*  PCSX2 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
*  without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
*  PURPOSE.  See the GNU General Public License for more details.
*
*  You should have received a copy of the GNU General Public License along with PCSX2.
*  If not, see <http://www.gnu.org/licenses/>.
*/

#include "PrecompiledHeader.h"

#include "GDBThread.h"

#include "SysThreads.h"

#include "DebugInterface.h"
#include "Breakpoints.h"
#include "DisassemblyManager.h"

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <signal.h>
//#include <unistd.h>
#ifdef _WIN32
#include <ws2tcpip.h>
#include <iphlpapi.h>
#include <iphlpapi.h>
#else
#include <sys/select.h>
#include <sys/socket.h>
#include <netinet/in.h>
#endif
#include <stdarg.h>

#define DEBUG_GDB
#undef dbgprintf
#ifndef DEBUG_GDB
#define dbgprintf(...)
#else
#define dbgprintf Console.WriteLn
#endif

#define fail(msg)   \
{                   \
    dbgprintf(msg); \
    gdb_deinit();   \
    return;         \
}

#define failr(msg)  \
{                   \
    dbgprintf(msg); \
    gdb_deinit();   \
    return 0;       \
}

#define		GDB_BFR_MAX	10000
#define		GDB_MAX_BP	100

#define		GDB_STUB_START	'$'
#define		GDB_STUB_END	'#'
#define		GDB_STUB_ACK	'+'
#define		GDB_STUB_NAK	'-'

#ifdef _WIN32
#define SIGTRAP 5
#define	SIGTERM 15
#define SIGSTOP 23 // values listed for MIPS?
#define SIGCONT 25 // values listed for MIPS?
#define MSG_WAITALL  8
#endif

enum gdb_bp_type
{
    GDB_BP_TYPE_NONE = 0,
    GDB_BP_TYPE_X,
    GDB_BP_TYPE_R,
    GDB_BP_TYPE_W,
    GDB_BP_TYPE_A,
};

class gdb_stub
{
public:
    gdb_stub();
    ~gdb_stub();

    void gdb_init(u32 port);
    void gdb_deinit();

    int gdb_data_available();

    void gdb_handle_events();

    void gdb_signal(u32 s, u64 addr = -1, MemCheckCondition cond = MEMCHECK_NONE);

protected:
    bool connected;
    int sock = -1;
    struct sockaddr_in saddr_server, saddr_client;

    u8 cmd_bfr[GDB_BFR_MAX + 1];
    u32 cmd_len;

    u32 sig;
    u64 signal_addr;
    MemCheckCondition signal_cond;

    DisassemblyManager manager;

protected:
    void gdb_read_command();
    void gdb_parse_command();

    u8 gdb_read_byte();
    u8 gdb_calc_chksum();

    void gdb_reply(const char *reply);
    void gdb_nak();
    void gdb_ack();

    void gdb_handle_signal();

    void gdb_continue();

    void gdb_detach();

    void gdb_read_registers();
    void gdb_write_registers();

    void gdb_handle_set_thread();

    void gdb_kill();

    void gdb_read_mem();
    void gdb_write_mem();

    void gdb_read_register();
    void gdb_write_register();

    void gdb_handle_query();

    void gdb_step();

    void gdb_add_bp();
    void gdb_remove_bp();

    void gdb_pause();

    void gdb_bp_add(u32 type, u32 addr, u32 len);
    void gdb_bp_remove(u32 type, u32 addr, u32 len);
};

// --------------------------------------------------------------------------------------
//  GDBThread Implementations
// --------------------------------------------------------------------------------------
GDBThread::GDBThread(u32 port) :
port(port),
gdb_interface(nullptr)
{
}

void GDBThread::OnStart()
{
    Console.WriteLn("GDB thread: On Start");

    m_name = L"GDBThread";
    _parent::OnStart();
}

void GDBThread::ExecuteTaskInThread()
{
    int oldtype = 0;
    pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, &oldtype);

    Console.WriteLn("Starting GDB stub thread.");

    gdb_stub stub_interface;

    gdb_interface = &stub_interface;
    is_running = true;

    while (is_running)
    {
        if (!gdb_interface)
        {
            is_running = false;
            break;
        }

        if (r5900Debug.isAlive())
        {
            gdb_interface->gdb_init(port);

            r5900Debug.pauseCpu();

            while (is_running && (0 <= gdb_interface->gdb_data_available()))
            {
                gdb_interface->gdb_handle_events();

                Threading::Sleep(0);
            }

            gdb_interface->gdb_deinit();
        }

        Threading::Sleep(0);
    }

    gdb_interface = nullptr;

    Console.WriteLn("Terminating GDB stub thread.");
}

void GDBThread::OnCleanupInThread()
{
    Console.WriteLn("GDB thread: On Cleanup");

    is_running = false;
    if (gdb_interface)
    {
        gdb_interface->gdb_signal(SIGTERM);
        gdb_interface->gdb_deinit();
    }
    _parent::OnCleanupInThread();
}

void GDBThread::OnPause()
{
    u64 addr;
    MemCheckCondition cond;

    Console.WriteLn("GDB thread: On Pause");

    if (!gdb_interface)
        return;

    if (GetCoreThread().IsClosing())
    {
        gdb_interface->gdb_signal(SIGTERM);
        gdb_interface->gdb_deinit();
    }
    else if (CBreakPoints::GetBreakpointTriggered(addr, cond))
    {
        gdb_interface->gdb_signal(SIGTRAP, addr, cond);
    }
    else
    {
        gdb_interface->gdb_signal(SIGSTOP);
    }
}
void GDBThread::OnResume()
{
    Console.WriteLn("GDB thread: On Resume");

    if (!gdb_interface)
        return;

    if (GetCoreThread().IsClosing())
    {
        gdb_interface->gdb_signal(SIGTERM);
        gdb_interface->gdb_deinit();
    }
    else
    {
        gdb_interface->gdb_signal(SIGCONT);
    }
}



// private helpers
static u8 hex2char(u8 hex)
{
    if (hex >= '0' && hex <= '9')
        return hex - '0';
    else if (hex >= 'a' && hex <= 'f')
        return hex - 'a' + 0xa;
    else if (hex >= 'A' && hex <= 'F')
        return hex - 'A' + 0xa;

    dbgprintf("Invalid nibble: %c (%02x)\n", hex, hex);
    return 0;
}

static u8 nibble2hex(u8 n)
{
    n &= 0xf;
    if (n < 0xa)
        return '0' + n;
    else
        return 'A' + n - 0xa;
}

static void mem2hex(u8 *dst, u32 src, u32 len)
{
    u8 tmp;

    while (len-- > 0)
    {
        tmp = r5900Debug.read8(src++);

        *dst++ = nibble2hex(tmp >> 4);
        *dst++ = nibble2hex(tmp);
    }
}

static void hex2mem(u32 dst, u8 *src, u32 len)
{
    u8 tmp;

    while (len-- > 0)
    {
        tmp = hex2char(*src++) << 4;
        tmp |= hex2char(*src++);

        r5900Debug.write8(dst++, tmp);
    }
}

static void wbe32hex(u8 *p, u32 v)
{
    u32 i;

    for (i = 0; i < 8; i++)
        p[i] = nibble2hex(v >> (28 - 4 * i));
}

static void wle32hex(u8 *p, u32 v)
{
    u32 i;

    for (i = 0; i < 8; i++)
        p[i] = nibble2hex(v >> (4 * (i ^ 1)));
}

static u32 rbe32hex(u8 *p)
{
    u32 i;
    u32 res = 0;

    for (i = 0; i < 8; i++)
        res = (res << 4) | hex2char(p[i]);

    return res;
}

static u32 rle32hex(u8 *p)
{
    u32 i;
    u32 res = 0;

    for (i = 0; i < 8; i++)
        res = (res) | (hex2char(p[i]) << (4 * (i ^ 1)));

    return res;
}

// GDB stub interface
gdb_stub::gdb_stub() :
sock(-1),
sig(0),
connected(false)
{
    manager.setCpu(&r5900Debug);
}
gdb_stub::~gdb_stub()
{
    gdb_deinit();
}

void gdb_stub::gdb_init(u32 port)
{
    int tmpsock;
    socklen_t len = sizeof saddr_client;
    int on;
#ifdef _WIN32
    WSADATA init_data;
    WSAStartup(MAKEWORD(2, 2), &init_data);
#endif

    tmpsock = socket(AF_INET, SOCK_STREAM, 0);
    if (tmpsock == -1)
        fail("Failed to create gdb socket");

    on = 1;
    if (setsockopt(tmpsock, SOL_SOCKET, SO_REUSEADDR, (const char*)&on, sizeof on) < 0)
        fail("Failed to setsockopt");

    memset(&saddr_server, 0, sizeof saddr_server);
    saddr_server.sin_family = AF_INET;
    saddr_server.sin_port = htons(port);
    saddr_server.sin_addr.s_addr = INADDR_ANY;

    if (bind(tmpsock, (struct sockaddr *)&saddr_server, sizeof saddr_server) < 0)
        fail("Failed to bind gdb socket");

    if (listen(tmpsock, 1) < 0)
        fail("Failed to listen to gdb socket");

    sock = tmpsock;

    dbgprintf("Waiting for gdb to connect...\n");
    int result = 0;
    while ((result = gdb_data_available()) == 0)
    {
        Threading::Sleep(0);
    }

    if (result < 0)
    {
        gdb_deinit();
        return;
    }

    sock = accept(tmpsock, (struct sockaddr *)&saddr_client, &len);

    if (sock < 0)
    {
        wprintf(L"accept failed with error: %ld\n", WSAGetLastError());
        fail("Failed to accept gdb client");
    }
    dbgprintf("Client connected.\n");

    saddr_client.sin_addr.s_addr = ntohl(saddr_client.sin_addr.s_addr);
    /*if (((saddr_client.sin_addr.s_addr >> 24) & 0xff) != 127 ||
    ((saddr_client.sin_addr.s_addr >> 16) & 0xff) !=   0 ||
    ((saddr_client.sin_addr.s_addr >>  8) & 0xff) !=   0 ||
    ((saddr_client.sin_addr.s_addr >>  0) & 0xff) !=   1)
    fail("gdb: incoming connection not from localhost");
    */
    closesocket(tmpsock);

    connected = true;
}

void gdb_stub::gdb_deinit()
{
    if (sock == -1)
        return;

    connected = false;

    shutdown(sock, SD_BOTH);

    closesocket(sock);
    sock = -1;
#ifdef _WIN32
    WSACleanup();
#endif
}

int gdb_stub::gdb_data_available()
{
    struct timeval t;
    fd_set _fds, *fds = &_fds;

    FD_ZERO(fds);
    FD_SET(sock, fds);

    t.tv_sec = 0;
    t.tv_usec = 20;

    if (select(sock + 1, fds, NULL, NULL, &t) < 0)
        return -1;

    if (FD_ISSET(sock, fds))
        return 1;
    return 0;
}

void gdb_stub::gdb_handle_events()
{
    if (!connected)
        return;

    while (0 < gdb_data_available())
    {
        gdb_read_command();
        gdb_parse_command();
    }
}

void gdb_stub::gdb_read_command(void)
{
    u8 c;
    u8 chk_read, chk_calc;

    cmd_len = 0;
    memset(cmd_bfr, 0, sizeof cmd_bfr);

    c = gdb_read_byte();
    if (c != GDB_STUB_START)
    {
        dbgprintf("gdb: read invalid byte %02x\n", c);
        return;
    }

    while ((c = gdb_read_byte()) != GDB_STUB_END)
    {
        cmd_bfr[cmd_len++] = c;
        if (cmd_len == sizeof cmd_bfr)
            fail("gdb: cmd_bfr overflow\n");
    }

    chk_read = hex2char(gdb_read_byte()) << 4;
    chk_read |= hex2char(gdb_read_byte());

    chk_calc = gdb_calc_chksum();

    if (chk_calc != chk_read)
    {
        dbgprintf("gdb: invalid checksum: calculated %02x and read %02x for $%s# (length: %d)\n", chk_calc, chk_read, cmd_bfr, cmd_len);
        cmd_len = 0;

        gdb_nak();
    }

    dbgprintf("gdb: read command %c with a length of %d: %s\n", cmd_bfr[0], cmd_len, cmd_bfr);
}

void gdb_stub::gdb_parse_command(void)
{
    if (cmd_len == 0)
        return;

    switch (cmd_bfr[0])
    {
    case 'q':
        gdb_handle_query();
        break;
    case 'H':
        gdb_handle_set_thread();
        break;
    case '?':
        gdb_handle_signal();
        break;
    case 'D':
        gdb_detach();
        break;
    case 'k':
        gdb_kill();
        break;
    case 'g':
        gdb_read_registers();
        break;
    case 'G':
        gdb_write_registers();
        break;
    case 'p':
        gdb_read_register();
        break;
    case 'P':
        gdb_write_register();
        break;
    case 'm':
        gdb_read_mem();
        break;
    case 'M':
        gdb_write_mem();
        break;
    case 'c':
        gdb_continue();
        break;
    case 's':
        gdb_step();
        break;
    case ' ':
        gdb_pause();
        break;
    case 'z':
        gdb_remove_bp();
        break;
    case 'Z':
        gdb_add_bp();
        break;
    default:
        gdb_ack();
        gdb_reply("");
        break;
    }
}

u8 gdb_stub::gdb_read_byte(void)
{
    if (!connected)
        return 0;

    size_t res;
    u8 c;

    res = recv(sock, (char*)&c, 1, MSG_WAITALL);
    if (res != 1)
        failr("recv failed");

    return c;
}

u8 gdb_stub::gdb_calc_chksum(void)
{
    u32 len = cmd_len;
    u8 *ptr = cmd_bfr;
    u8 c = 0;

    while (len-- > 0)
        c += *ptr++;

    return c;
}

void gdb_stub::gdb_reply(const char *reply)
{
    if (!connected)
        return;

    u8 chk;
    u32 left;
    u8 *ptr;
    int n;

    memset(cmd_bfr, 0, sizeof cmd_bfr);

    cmd_len = strlen(reply);
    if (cmd_len + 4 > sizeof cmd_bfr)
        fail("cmd_bfr overflow in gdb_reply");

    memcpy(cmd_bfr + 1, reply, cmd_len);

    cmd_len++;
    chk = gdb_calc_chksum();
    cmd_len--;
    cmd_bfr[0] = GDB_STUB_START;
    cmd_bfr[cmd_len + 1] = GDB_STUB_END;
    cmd_bfr[cmd_len + 2] = nibble2hex(chk >> 4);
    cmd_bfr[cmd_len + 3] = nibble2hex(chk);

    dbgprintf("gdb: reply (len: %d): %s\n", cmd_len, cmd_bfr);

    ptr = cmd_bfr;
    left = cmd_len + 4;
    while (left > 0)
    {
        n = send(sock, (const char*)ptr, left, 0);
        if (n < 0)
            fail("gdb: send failed");
        left -= n;
        ptr += n;
    }
}

void gdb_stub::gdb_nak(void)
{
    if (!connected)
        return;

    const char nak = GDB_STUB_NAK;
    size_t res;

    res = send(sock, &nak, 1, 0);
    if (res != 1)
        fail("send failed");
}

void gdb_stub::gdb_ack(void)
{
    if (!connected)
        return;

    const char ack = GDB_STUB_ACK;
    size_t res;

    res = send(sock, &ack, 1, 0);
    if (res != 1)
        fail("send failed");
}

void gdb_stub::gdb_handle_signal(void)
{
    if (!connected)
        return;

    char bfr[128] = { 0 };

    gdb_ack();

    memset(bfr, 0, sizeof bfr);
    switch (signal_cond)
    {
    case MEMCHECK_NONE:
        sprintf(bfr, "T%02X", sig);
        break;
    case MEMCHECK_READ:
        sprintf(bfr, "T%02Xrwatch:%08llX", sig, signal_addr);
        break;
    case MEMCHECK_WRITE:
        sprintf(bfr, "T%02Xwatch:%08llX", sig, signal_addr);
        break;
    case MEMCHECK_READWRITE:
        sprintf(bfr, "T%02Xawatch:%08llX", sig, signal_addr);
        break;
    default:
        return;
        break;
    }

    gdb_reply(bfr);
}

void gdb_stub::gdb_continue(void)
{
    if (!connected)
        return;

    gdb_ack();

    if (r5900Debug.isCpuPaused())
    {
        // If the current PC is on a breakpoint, the user doesn't want to do nothing.
        CBreakPoints::SetSkipFirst(r5900Debug.getPC());
        r5900Debug.resumeCpu();
    }
}

void gdb_stub::gdb_detach(void)
{
    if (!connected)
        return;

    gdb_ack();
    gdb_reply("OK");
    gdb_deinit();
}

void gdb_stub::gdb_read_registers(void)
{
    if (!connected)
        return;

    u8 bfr[GDB_BFR_MAX - 4] = { 0 };
    u32 i, j, reg;

    gdb_ack();
    memset(bfr, 0, sizeof bfr);

    for (i = 0, reg = 0; i < EECAT_COUNT; ++i)
    {
        int cnt = r5900Debug.getRegisterCount(i);
        for (j = 0; j < cnt; ++j, ++reg)
        {
            u128 value = r5900Debug.getRegister(i, j);

            wbe32hex(bfr + reg * 32 + 0,  value._u32[0]);
            wbe32hex(bfr + reg * 32 + 8,  value._u32[1]);
            wbe32hex(bfr + reg * 32 + 16, value._u32[2]);
            wbe32hex(bfr + reg * 32 + 24, value._u32[3]);
        }
    }

    gdb_reply((char *)bfr);
}

void gdb_stub::gdb_write_registers(void)
{
    if (!connected)
        return;

    gdb_ack();

    u32 i, j, reg;

    for (i = 0, reg = 0; i < EECAT_COUNT; ++i)
    {
        int cnt = r5900Debug.getRegisterCount(i);
        for (j = 0; j < cnt; ++j, ++reg)
        {
            u128 value;

            value._u32[0] = rbe32hex(cmd_bfr + reg * 32 + 0);
            value._u32[1] = rbe32hex(cmd_bfr + reg * 32 + 8);
            value._u32[2] = rbe32hex(cmd_bfr + reg * 32 + 16);
            value._u32[3] = rbe32hex(cmd_bfr + reg * 32 + 24);

            r5900Debug.setRegister(i, j, value);
        }
    }

    gdb_reply("OK");
}

void gdb_stub::gdb_handle_set_thread(void)
{
    if (!connected)
        return;

    gdb_ack();
    if (memcmp(cmd_bfr, "Hg0", 3) == 0 ||
        memcmp(cmd_bfr, "Hc-1", 4) == 0)
        return gdb_reply("OK");
    gdb_reply("E01");
}

void gdb_stub::gdb_kill(void)
{
    if (!connected)
        return;

    gdb_ack();
    fail("killed by gdb");
}

void gdb_stub::gdb_read_mem(void)
{
    if (!connected)
        return;

    u8 reply[GDB_BFR_MAX - 4] = { 0 };
    u32 addr, len;
    u32 i;

    gdb_ack();

    i = 1;
    addr = 0;
    while (cmd_bfr[i] != ',')
        addr = (addr << 4) | hex2char(cmd_bfr[i++]);

    i++;

    len = 0;
    while (i < cmd_len)
        len = (len << 4) | hex2char(cmd_bfr[i++]);
    dbgprintf("gdb: read memory: %08x bytes from %08x\n", len, addr);

    if (len * 2 > sizeof reply)
        gdb_reply("E01");

    mem2hex(reply, addr, len);
    gdb_reply((char *)reply);
}

void gdb_stub::gdb_write_mem(void)
{
    if (!connected)
        return;

    u32 addr, len;
    u32 i;

    gdb_ack();

    i = 1;
    addr = 0;
    while (cmd_bfr[i] != ',')
        addr = (addr << 4) | hex2char(cmd_bfr[i++]);

    i++;

    len = 0;
    while (cmd_bfr[i] != ':')
        len = (len << 4) | hex2char(cmd_bfr[i++]);
    dbgprintf("gdb: write memory: %08x bytes to %08x\n", len, addr);

    hex2mem(addr, cmd_bfr + i, len);
    gdb_reply("OK");
}

void gdb_stub::gdb_read_register(void)
{
    if (!connected)
        return;

    u8 reply[32] = { 0 };
    u32 id = 0;

    gdb_ack();

    int i = 1;
    while (i < cmd_len)
        id = (id << 4) | hex2char(cmd_bfr[i++]);

    int category = ((id >> 8) & 0xFF);
    int index = (id & 0xFF);

    u128 value;

    value = r5900Debug.getRegister(category, index);

    wbe32hex(reply + 0, value._u32[0]);
    wbe32hex(reply + 8, value._u32[1]);
    wbe32hex(reply + 16, value._u32[2]);
    wbe32hex(reply + 24, value._u32[3]);

    gdb_reply((char *)reply);
}

void gdb_stub::gdb_write_register(void)
{
    if (!connected)
        return;

    u32 id = 0;

    gdb_ack();

    int i = 1;
    while (cmd_bfr[i] != '=')
        id = (id << 4) | hex2char(cmd_bfr[i++]);
    ++i;

    int category = ((id >> 8) & 0xFF);
    int index = (id & 0xFF);

    u128 value;

    value._u32[0] = rbe32hex(cmd_bfr + i + 0);
    value._u32[1] = rbe32hex(cmd_bfr + i + 8);
    value._u32[2] = rbe32hex(cmd_bfr + i + 16);
    value._u32[3] = rbe32hex(cmd_bfr + i + 24);

    r5900Debug.setRegister(category, index, value);

    gdb_reply("OK");
}

void gdb_stub::gdb_handle_query(void)
{
    if (!connected)
        return;

    dbgprintf("gdb: query '%s'\n", cmd_bfr + 1);
    gdb_ack();
    gdb_reply("");
}

void gdb_stub::gdb_step(void)
{
    if (!connected)
        return;

    gdb_ack();

    if (!r5900Debug.isAlive() || !r5900Debug.isCpuPaused())
        return;

    // If the current PC is on a breakpoint, the user doesn't want to do nothing.
    u32 currentPc = r5900Debug.getPC();
    CBreakPoints::SetSkipFirst(currentPc);

    MIPSAnalyst::MipsOpcodeInfo info = MIPSAnalyst::GetOpcodeInfo(&r5900Debug, currentPc);
    u32 breakpointAddress = currentPc + manager.getInstructionSizeAt(currentPc);
    if (info.isBranch)
    {
        if (info.isConditional == false)
        {
            breakpointAddress = info.branchTarget;
        }
        else
        {
            if (info.conditionMet)
            {
                breakpointAddress = info.branchTarget;
            }
            else
            {
                breakpointAddress = currentPc + 2 * 4;
            }
        }
    }

    if (info.isSyscall)
        breakpointAddress = info.branchTarget;

    CBreakPoints::AddBreakPoint(breakpointAddress, true);
    r5900Debug.resumeCpu();
}

void gdb_stub::gdb_add_bp(void)
{
    if (!connected)
        return;

    u32 type = 0, addr = 0, len = 0, i = 1;

    gdb_ack();

    while (cmd_bfr[i] != ',')
        type = (type << 4) | hex2char(cmd_bfr[i++]);
    i++;

    switch (type)
    {
    case 0:
    case 1:
        type = GDB_BP_TYPE_X;
        break;
    case 2:
        type = GDB_BP_TYPE_W;
        break;
    case 3:
        type = GDB_BP_TYPE_R;
        break;
    case 4:
        type = GDB_BP_TYPE_A;
        break;
    default:
        return gdb_reply("E01");
    }

    addr = 0;
    len = 0;

    while (cmd_bfr[i] != ',')
        addr = (addr << 4) | hex2char(cmd_bfr[i++]);
    i++;

    while (i < cmd_len)
        len = (len << 4) | hex2char(cmd_bfr[i++]);

    gdb_bp_add(type, addr, len);
    gdb_reply("OK");
}

void gdb_stub::gdb_remove_bp(void)
{
    if (!connected)
        return;

    u32 type = 0, addr = 0, len = 0, i = 1;

    gdb_ack();

    while (cmd_bfr[i] != ',')
        type = (type << 4) | hex2char(cmd_bfr[i++]);
    i++;

    switch (type)
    {
    case 0:
    case 1:
        type = GDB_BP_TYPE_X;
        break;
    case 2:
        type = GDB_BP_TYPE_W;
        break;
    case 3:
        type = GDB_BP_TYPE_R;
        break;
    case 4:
        type = GDB_BP_TYPE_A;
        break;
    default:
        return gdb_reply("E01");
    }

    addr = 0;
    len = 0;

    while (cmd_bfr[i] != ',')
        addr = (addr << 4) | hex2char(cmd_bfr[i++]);
    i++;

    while (i < cmd_len)
        len = (len << 4) | hex2char(cmd_bfr[i++]);

    gdb_bp_remove(type, addr, len);
    gdb_reply("OK");
}

void gdb_stub::gdb_pause(void)
{
    if (!connected)
        return;

    gdb_ack();

    if (!r5900Debug.isCpuPaused())
    {
        r5900Debug.pauseCpu();
    }
}

void gdb_stub::gdb_signal(u32 s, u64 addr, MemCheckCondition cond)
{
    sig = s;
    signal_addr = addr;
    signal_cond = cond;

    gdb_handle_signal();
}

void gdb_stub::gdb_bp_add(u32 type, u32 addr, u32 len)
{
    MemCheckCondition condition;
    MemCheckResult result = MEMCHECK_BOTH;
    bool is_mem_check = false;

    switch (type)
    {
    case GDB_BP_TYPE_X:
    {
        is_mem_check = false;
    }
    break;
    case GDB_BP_TYPE_W:
    {
        is_mem_check = true;
        condition = MEMCHECK_WRITE;
    }
    break;
    case GDB_BP_TYPE_R:
    {
        is_mem_check = true;
        condition = MEMCHECK_READ;
    }
    break;
    case GDB_BP_TYPE_A:
    {
        is_mem_check = true;
        condition = MEMCHECK_READWRITE;
    }
    break;
    }

    if (is_mem_check)
    {
        CBreakPoints::AddMemCheck(addr, addr + len, condition, result);
    }
    else
    {
        CBreakPoints::AddBreakPoint(addr);
    }

    dbgprintf("gdb: added a %d breakpoint: %08x bytes at %08X\n", type, len, addr);
}

void gdb_stub::gdb_bp_remove(u32 type, u32 addr, u32 len)
{
    bool is_mem_check = false;

    switch (type)
    {
    case GDB_BP_TYPE_X:
    {
        is_mem_check = false;
    }
    break;
    case GDB_BP_TYPE_W:
    case GDB_BP_TYPE_R:
    case GDB_BP_TYPE_A:
    {
        is_mem_check = true;
    }
    break;
    }

    if (is_mem_check)
    {
        CBreakPoints::RemoveMemCheck(addr, addr + len);
    }
    else
    {
        CBreakPoints::RemoveBreakPoint(addr);
    }

    dbgprintf("gdb: removed a %d breakpoint: %08x bytes at %08X\n", type, len, addr);
}
