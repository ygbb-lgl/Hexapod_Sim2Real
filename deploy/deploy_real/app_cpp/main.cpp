/*
 * @Description:
 * @Author: kx zhang
 * @Date: 2022-09-13 19:00:55
 * @LastEditTime: 2022-11-13 17:09:03
 */

#include <cstdio>
#include <readline/readline.h>
#include <map>

#include "Console.hpp"
#include "command.h"
#include "time.h"

extern "C" {
#include "ethercat.h"
}

namespace cr = CppReadline;
using ret = cr::Console::ReturnCode;
std::thread comThread;


void comImpl() {
    cr::Console c(">");
    
    std::map<std::string, cr::Console::CommandFunction> commands = {
        {"help", help},
        {"MotorIdGet", motorIdGet},
        {"MotorIdSet", motorIdSet},
        {"MotorIdReset", motorIdReset},
        {"MotorZeroSet", motorZeroSet},
        {"MotorStop", motorStop},
        {"MotorSpeedSet", motorSpeedSet},
        {"MotorPositionSet", motorPositionSet}
    };

    for (const auto& cmd : commands) {
        c.registerCommand(cmd.first, cmd.second);
    }

    c.executeCommand("help");
    int retCode;
    do {
        retCode = c.readLine();
        // We can also change the prompt based on last return value:
        if (retCode == ret::Ok)
            c.setGreeting(">");
        else
            c.setGreeting("!>");

        if (retCode == 1) {
            std::cout << "Received error code 1\n";
        } else if (retCode == 2) {
            std::cout << "Received error code 2\n";
        }

        usleep(100000);
        std::cout << std::endl;
    } while (retCode != ret::Quit);
}

int main() {
    printf("SOEM 主站测试\n");

    // 这里填自己电脑上的网卡
    EtherCAT_Init((char *) "enp109s0"); //ens33:

    if (ec_slavecount <= 0) {
        printf("未找到从站, 程序退出！\n");
        return 1;
    } else
        printf("从站数量： %d\n", ec_slavecount);

    startRun();

    comThread = std::thread(comImpl);
    comThread.join();

    runThread.join();
    return 0;
}

