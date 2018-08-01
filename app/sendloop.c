#include <stdio.h>
#include <rtt_stdio.h>
#include "ot.h"
#include <openthread/openthread.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>

#include "sched.h"
#include <periph/cpuid.h>

#include "target.h"

#define ECHO_BUF_SIZE 4
char echo_buf[ECHO_BUF_SIZE];

extern char cpu_id[CPUID_LEN];
extern char ip6_addr[CPUID_LEN];

void send_echo_loop(void) {
    int sock;
    int rv;
    for (;;) {
        {
            sock = socket(AF_INET6, SOCK_STREAM, 0);
            if (sock == -1) {
                perror("socket");
                goto retry;
            }

            struct sockaddr_in6 receiver;
            receiver.sin6_family = AF_INET6;
            receiver.sin6_port = htons(RECEIVER_PORT);

            rv = inet_pton(AF_INET6, TARGET_IP6_ADDR, &receiver.sin6_addr);
            if (rv == -1) {
                perror("invalid address family in inet_pton");
                goto retry;
            } else if (rv == 0) {
                perror("invalid ip address in inet_pton");
                goto retry;
            }

            struct sockaddr_in6 local;
            local.sin6_family = AF_INET6;
            local.sin6_addr = in6addr_any;
            local.sin6_port = htons(SENDER_PORT);

            rv = bind(sock, (struct sockaddr*) &local, sizeof(struct sockaddr_in6));
            if (rv == -1) {
                perror("bind");
                goto retry;
            }

            printf("Calling connect()\n");

            rv = connect(sock, (struct sockaddr*) &receiver, sizeof(struct sockaddr_in6));
            if (rv == -1) {
                perror("connect");
                goto retry;
            }

            printf("Sucessful connect()\n");
        }

        for (;;) {
            rv = send(sock, cpu_id, CPUID_SEND_LEN, 0);
            if (rv == -1) {
                perror("send");
                goto retry;
            }
            rv = send(sock, ip6_addr, sizeof(ip6_addr), 0);
            if (rv == -1) {
                perror("send");
                goto retry;
            }
            rv = send(sock, echo_buf, ECHO_BUF_SIZE, 0);
            if (rv == -1) {
                perror("send");
                goto retry;
            }

            int bytes_read = 0;
            while (bytes_read < ECHO_BUF_SIZE) {
                rv = recv(sock, &echo_buf[bytes_read], ECHO_BUF_SIZE - bytes_read, 0);
                if (rv == 0) {
                    goto retry;
                }
                if (rv < 0) {
                    perror("read");
                    goto retry;
                }
                bytes_read += rv;
            }

            /* Blink LED and wait 20 seconds before replying. */
            LED_ON;
            xtimer_usleep(500000u);
            LED_OFF;
            xtimer_usleep(19500000u);
        }

    retry:
        close(sock);
        /* Wait three seconds before trying to connect again. */
        xtimer_usleep(3000000u);
    }
}

void* sendloop(void* arg) {
    (void) arg;
    send_echo_loop();
    return NULL;
}

static kernel_pid_t sendloop_pid = 0;
//static char sendloop_stack[1392];
static char sendloop_stack[2500] __attribute__((aligned(4)));
kernel_pid_t start_sendloop(void)
{
    if (sendloop_pid != 0) {
        return sendloop_pid;
    }
    otIp6AddUnsecurePort(openthread_get_instance(), SENDER_PORT);
    sendloop_pid = thread_create(sendloop_stack, sizeof(sendloop_stack),
                                 THREAD_PRIORITY_MAIN - 1,
                                 THREAD_CREATE_STACKTEST, sendloop, NULL,
                                 "sendloop");
    return sendloop_pid;
}
