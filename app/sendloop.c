#include <stdio.h>
#include <rtt_stdio.h>
#include "ot.h"
#include <openthread/openthread.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>

#include "sched.h"
#include <cond.h>
#include <mutex.h>

#include "anemometer.h"

measure_set_t reading;
bool reading_ready = false;
mutex_t ready_mutex = MUTEX_INIT;
cond_t ready_cond = COND_INIT;

void send_measurement_loop(void) {
    for (;;) {
        int sock = socket(AF_INET6, SOCK_STREAM, 0);
        if (sock == -1) {
            perror("socket");
            goto retry;
        }

        struct sockaddr_in6 receiver;
        receiver.sin6_family = AF_INET6;
        receiver.sin6_port = htons(RECEIVER_PORT);

        int rv = inet_pton(AF_INET6, RECEIVER_IP, &receiver.sin6_addr);
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

        rv = connect(sock, (struct sockaddr*) &receiver, sizeof(struct sockaddr_in6));
        if (rv == -1) {
            perror("connect");
            goto retry;
        }

        for (;;) {
            measure_set_t local_reading;
            /*
             * This loop does the following:
             * 1) Wait until the next measurement is ready
             * 2) Send it over TCP
             */
            mutex_lock(&ready_mutex);
            while (!reading_ready) {
                cond_wait(&ready_cond, &ready_mutex);
            }
            memcpy(&local_reading, &reading, sizeof(local_reading));
            reading_ready = false;
            mutex_unlock(&ready_mutex);

            /*
             * Now we have copied the reading into our local space, so we don't
             * need to worry about the measurement thread overwriting it.
             */
            rv = send(sock, &local_reading, sizeof(local_reading), 0);
            if (rv == -1) {
                perror("send");
                goto retry;
            }
        }

    retry:
        close(sock);
    }
}

void* sendloop(void* arg) {
    (void) arg;
    send_measurement_loop();
    return NULL;
}

static kernel_pid_t sendloop_pid = 0;
static char sendloop_stack[2048];
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
