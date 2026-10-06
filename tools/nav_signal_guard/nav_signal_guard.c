/* nav_signal_guard.c — 让导航容器不响应 SIGINT / SIGTERM（只留 SIGKILL）
 *
 * 背景（5 份 core 一致）：
 *   component_container_isolated 一旦处理 SIGINT/SIGTERM，就执行 Nav2 生命周期清理
 *   （controller_server 析构 RotationShim(MPPI) + 层叠 costmap 清理），随后在
 *   librclcpp.so+0x433d0 虚调用打空 → SIGSEGV(exit -11) → 导航栈整体消失。
 *   09-29（17:00/17:28/17:39）与 10-06 12:50 的触发信号来源不一，且 Linux 不记录
 *   信号发送者，故采用“容器对 SIGINT/SIGTERM 免疫”的做法。
 *
 * 机制（关键）：
 *   仅用 sigaction(SIG_IGN) 不够——rclcpp 初始化时会安装自己的信号处理函数，
 *   覆盖掉忽略设置（实测容器仍打印 signal_handler(SIGINT/SIGTERM) 并崩溃）。
 *   这里改为**永久屏蔽信号掩码**：SIGINT/SIGTERM 被阻塞且从不解除、也不安装
 *   处理函数，因此进程永远不会“投递”这两个信号。SIGKILL 无法被屏蔽，
 *   所以停止仍然有效：ros2 launch 包装与桥的停止阶梯最终以 SIGKILL 收尾。
 *
 * 关闭方式：NAV_SIGNAL_GUARD=0 → 不屏蔽，恢复原行为。
 * 编译：gcc -shared -fPIC -O2 -o nav_signal_guard.so nav_signal_guard.c
 */
#define _GNU_SOURCE
#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>

__attribute__((constructor))
static void nav_signal_guard_init(void)
{
    const char *flag = getenv("NAV_SIGNAL_GUARD");
    if (flag != NULL && (strcmp(flag, "0") == 0 || strcmp(flag, "false") == 0)) {
        return;   /* 显式关闭 */
    }

    const char *log_path = getenv("NAV_SIGNAL_GUARD_LOG");
    if (log_path == NULL || log_path[0] == '\0') {
        log_path = "/tmp/nav_signal_guard.log";
    }

    /* 1) 用 sigaction 兜一层（对不重置处理函数的程序立即生效） */
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = SIG_IGN;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
    sigaction(SIGHUP, &sa, NULL);

    /* 2) 关键：永久屏蔽 SIGINT/SIGTERM/SIGHUP（阻塞掩码不可被 SIG_IGN 覆盖） */
    sigset_t blocked;
    sigemptyset(&blocked);
    sigaddset(&blocked, SIGINT);
    sigaddset(&blocked, SIGTERM);
    sigaddset(&blocked, SIGHUP);
    int rc = sigprocmask(SIG_BLOCK, &blocked, NULL);

    FILE *fp = fopen(log_path, "a");
    if (fp != NULL) {
        fprintf(fp, "pid=%d guard armed (SIGINT/SIGTERM masked, rc=%d)\n",
                (int)getpid(), rc);
        fclose(fp);
    }
}
