#include <stdio.h>
#include <rtt_stdio.h>
#include "shell.h"
#include "thread.h"
#include "xtimer.h"
#include <string.h>
#include "msg.h"
#include "net/gnrc.h"
#include "net/gnrc/ipv6.h"
#include "net/gnrc/udp.h"
#include "net/gnrc/netapi.h"
#include "net/gnrc/netreg.h"
#include "phydat.h"
#include "saul_reg.h"
#include <periph/cpuid.h>
#include <periph/gpio.h>
#include "mutex.h"
#include "cond.h"
#include "ot.h"
#include "target.h"

#include <openthread/udp.h>
#include <openthread/cli.h>
#include <openthread/openthread.h>

#define ENABLE_DEBUG (1)
#include "debug.h"

/* I need these variables to make OpenThread compile. */
uint16_t myRloc = 0;

#ifdef CPU_DUTYCYCLE_MONITOR
volatile uint64_t cpuOnTime = 0;
volatile uint64_t cpuOffTime = 0;
volatile uint32_t contextSwitchCnt = 0;
volatile uint32_t preemptCnt = 0;
#endif
#ifdef RADIO_DUTYCYCLE_MONITOR
uint64_t radioOnTime = 0;
uint64_t radioOffTime = 0;
#endif

uint32_t packetSuccessCnt = 0;
uint32_t packetFailCnt = 0;
uint32_t packetBusyChannelCnt = 0;
uint32_t broadcastCnt = 0;
uint32_t queueOverflowCnt = 0;

uint16_t nextHopRloc = 0;
uint8_t borderRouterLC = 0;
uint8_t borderRouterRC = 0;
uint32_t routeChangeCnt = 0;
uint32_t borderRouteChangeCnt = 0;

uint32_t totalIpv6MsgCnt = 0;
uint32_t Ipv6TxSuccCnt = 0;
uint32_t Ipv6TxFailCnt = 0;
uint32_t Ipv6RxSuccCnt = 0;
uint32_t Ipv6RxFailCnt = 0;

uint32_t pollMsgCnt = 0;
uint32_t mleMsgCnt = 0;

uint32_t mleRouterMsgCnt = 0;
uint32_t addrMsgCnt = 0;
uint32_t netdataMsgCnt = 0;

uint32_t meshcopMsgCnt = 0;
uint32_t tmfMsgCnt = 0;

uint32_t totalSerialMsgCnt = 0;

static otUdpSocket mSocket;
static otMessageInfo messageInfo;
static otMessage *message = NULL;

#define OPENTHREAD_INIT_TIME 5000000ul

char cpu_id[CPUID_LEN];
char ip6_addr[16];

char udp_buffer[CPUID_SEND_LEN + 16 + 4];

void on_received_udp(void* aContext, otMessage* aMessage, const otMessageInfo* aMessageInfo) {
    (void) aContext;
    (void) aMessageInfo;

    if (aMessage == NULL) {
        return;
    }

    uint16_t offset = otMessageGetOffset(aMessage);
    uint16_t length = otMessageGetLength(aMessage);
    if (offset - length == 4) {
        otMessageRead(aMessage, offset, &udp_buffer[CPUID_SEND_LEN + 16], 4);
    }
}


int main(void)
{
    cpuid_get(cpu_id);
    xtimer_usleep(OPENTHREAD_INIT_TIME);

    {
        const otIp6Address* address;

        openthread_lock_coarse_mutex();
        address = otThreadGetMeshLocalEid(openthread_get_instance());
        memcpy(ip6_addr, address, sizeof(ip6_addr));
        openthread_unlock_coarse_mutex();
    }

    memcpy(&udp_buffer[0], cpu_id, CPUID_SEND_LEN);
    memcpy(&udp_buffer[CPUID_SEND_LEN], ip6_addr, 16);

    start_sendloop();

    /* Send/receive UDP packets. */

    openthread_lock_coarse_mutex();

    /* Prepare for sending/receiving UDP packets */
    otError error;
    {
        otSockAddr addr;
        //memcpy(&addr.mAddress, ip6_addr, sizeof(addr.mAddress));
        memset(&addr.mAddress, 0x00, sizeof(addr.mAddress)); // Unspecifed
        addr.mPort = 12345;
        addr.mScopeId = 0; // All interfaces
        otUdpOpen(openthread_get_instance(), &mSocket, on_received_udp, NULL);
        otUdpBind(&mSocket, &addr);
    }

    memset(&messageInfo, 0x00, sizeof(messageInfo));
    error = otIp6AddressFromString(TARGET_IP6_ADDR, &messageInfo.mPeerAddr);
    if (error != OT_ERROR_NONE) {
        DEBUG("error in otIp6AddressFromString\n");
    }
    //otIp6AddressFromString("fdde:ad00:beef:0000:e9f0:45bc:c507:6f0e", &messageInfo.mPeerAddr);
    messageInfo.mPeerPort = RECEIVER_PORT;
    messageInfo.mInterfaceId = 1;

    openthread_unlock_coarse_mutex();

    for (;;) {
        openthread_lock_coarse_mutex();

        /* Send a UDP packet periodically. */
        message = otUdpNewMessage(openthread_get_instance(), false);
        if (message == NULL) {
            DEBUG("error in new message");
        } else {
            int message_size = CPUID_SEND_LEN + 16 + 4;
            error = otMessageSetLength(message, message_size);
            if (error != OT_ERROR_NONE) {
                DEBUG("error in set length\n");
            }
            otMessageWrite(message, 0, udp_buffer, message_size);

            DEBUG("\n[Main] Tx UDP packet\n");
            error = otUdpSend(&mSocket, message, &messageInfo);
            if (error != OT_ERROR_NONE) {
                DEBUG("error in udp send\n");
                otMessageFree(message);
            }
        }

        openthread_unlock_coarse_mutex();

        /* Sleep for 20 seconds. */
        xtimer_usleep(20000000u);
    }

    return 0;
}
