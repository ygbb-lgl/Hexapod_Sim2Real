//
// Created by bismarck on 11/19/22.
//

#include "command.h"

void sendToQueue(int slaveId, const Queue_Msg_ptr& msg) {

    if (messages[slaveId].write_available()) {
        messages[slaveId].push(msg);
    } else {
        std::cout << "Queue Fulled, Waiting For Command Executing\n";
        while(!messages[slaveId].push(msg)) sleep(1);
    }

}

Queue_Msg_ptr createQueueMsg(EtherCAT_Msg_ptr& msg, uint8_t passage) {
    Queue_Msg_ptr queue_msg = std::make_shared<Queue_Msg>();
    queue_msg->passage = passage;
    queue_msg->motor = msg->motor[queue_msg->passage - 1];
    return queue_msg;
}


unsigned help(const std::vector<std::string> &) {
    std::cout << "Available Commands:\n"
        << "\tMotorIdGet <SlaveId>\n"
        << "\tMotorIdSet <SlaveId> <MotorId> <NewMotorId>\n"
        << "\tMotorIdReset <SlaveId>\n"
        << "\tMotorZeroSet <SlaveId> <PassAge> <MotorId>\n"
        << "\tMotorStop <SlaveId> <PassAge> <MotorId>\n"
        << "\tMotorSpeedSet <SlaveId> <PassAge> <MotorId> <Speed>(0) <Current>(500) <AckStatus>(2)\n"
        << "\tMotorPositionSet <SlaveId> <PassAge> <MotorId> <Position>(0) <Speed>(50) <Current>(500) <AckStatus>(2)\n";
    return 0;
}

unsigned motorIdGet(const std::vector<std::string> & input) {
    int slaveId;
    switch (input.size()-1) {
        case 1:
            slaveId = std::stoi(input[1]);
            break;
        default:
            std::cout << "Command format error\n" <<
                      "\tShould be \"MotorIdGet <SlaveId>\"\n";
            return 1;
    }


    EtherCAT_Msg_ptr msg = std::make_shared<EtherCAT_Msg>();
    MotorIDReading(msg.get());
    Queue_Msg_ptr queue_msg = createQueueMsg(msg, 1);
    sendToQueue(slaveId, queue_msg);
    return 0;
}

unsigned motorIdSet(const std::vector<std::string> & input) {
    int slaveId;
    int motor_id,  motor_id_new;
    try {
        switch (input.size()-1) {
            case 3:
                slaveId = std::stoi(input[1]);
                motor_id = std::stoi(input[2]);
                motor_id_new = std::stoi(input[3]);
                break;
            default:
                std::cout << "Command format error\n" <<
                          "\tShould be \"MotorIdSet <SlaveId> <OldMotorId> <NewMotorId>\"\n";
                return 1;
        }
    } catch (const std::exception& e) {
        std::cout << "Parameter error" << e.what() << '\n';
        return 1;
    }

    EtherCAT_Msg_ptr msg = std::make_shared<EtherCAT_Msg>();
    MotorIDSetting(msg.get(), motor_id, motor_id_new);
    Queue_Msg_ptr queue_msg = createQueueMsg(msg, 1);
    sendToQueue(slaveId, queue_msg);
    return 0;
}

unsigned motorIdReset(const std::vector<std::string> & input) {
    int slaveId;
    try {
        switch (input.size() - 1) {
            case 1:
                slaveId = std::stoi(input[1]);
                break;
            default:
                std::cout << "Command format error\n" <<
                          "\tShould be \"MotorIdReset <SlaveId>\"\n";
                return 1;
        }
    } catch (const std::exception& e) {
        std::cout << "Parameter error" << e.what() << "\n";
        return 1;
    }
    EtherCAT_Msg_ptr msg = std::make_shared<EtherCAT_Msg>();
    MotorIDReset(msg.get());
    Queue_Msg_ptr queue_msg = createQueueMsg(msg, 1);
    sendToQueue(slaveId, queue_msg);
    return 0;
}

unsigned motorZeroSet(const std::vector<std::string> & input) {
    int slaveId;
    int passage;
    int motorId;
    try {
        switch (input.size() - 1) {
            case 3:
                slaveId = std::stoi(input[1]);
                passage = std::stoi(input[2]);
                motorId = std::stoi(input[3]);
                break;
            default:
                std::cout << "Command format error\n" <<
                          "\tShould be \"MotorZeroSet <SlaveId> <PassAge> <MotorId>\"\n";
                return 1;
        }
    } catch (const std::exception& e) {
        std::cout << "Parameter error" << e.what() << "\n";
        return 1;
    }
    EtherCAT_Msg_ptr msg = std::make_shared<EtherCAT_Msg>();

    Motor_Setzero(msg.get(), passage, motorId);

    Queue_Msg_ptr queue_msg = createQueueMsg(msg, passage);
    sendToQueue(slaveId, queue_msg);
    return 0;
}

unsigned motorSpeedSet(const std::vector<std::string> & input) {
    int slaveId;
    int motor_id ;
    float spd = 0;
    uint8_t passage;
    uint16_t cur = 500;
    uint8_t ack_status = 2;
    try {
        switch (input.size()-1) {
            case 6:
                ack_status = std::stoi(input[6]);
            case 5:
                cur = std::stoi(input[5]);
            case 4:
                spd = std::stof(input[4]);
            case 3:
                slaveId = std::stoi(input[1]);
                passage = std::stoi(input[2]);
                motor_id = std::stoi(input[3]);
                break;
            default:
                std::cout << "Command format error\n" <<
                          "\tShould be \"MotorSpeedSet <SlaveId> <PassAge> <OldMotorId> <Speed>(0) <Current>(500) <AckStatus>(2)\"\n";
                return 1;
        }
    } catch (const std::exception& e) {
        std::cout << "Parameter error" << e.what() << "\n";
        return 1;
    }



    EtherCAT_Msg_ptr msg = std::make_shared<EtherCAT_Msg>();
    set_motor_speed(msg.get(), passage ,motor_id, spd, cur, ack_status);
    Queue_Msg_ptr queue_msg = createQueueMsg(msg, passage);
    sendToQueue(slaveId, queue_msg);
    return 0;
}

unsigned motorPositionSet(const std::vector<std::string> & input) {
    int slaveId;
    int motor_id;
    float pos = 0;
    uint8_t passage;
    uint16_t spd = 50;
    uint16_t cur = 500;
    uint8_t ack_status = 2;
    try {
        switch (input.size()-1) {
            case 7:
                ack_status = std::stoi(input[7]);
            case 6:
                cur = std::stoi(input[6]);
            case 5:
                spd = std::stoi(input[5]);
            case 4:
                pos = std::stof(input[4]);
            case 3:
                slaveId = std::stoi(input[1]);
                passage = std::stoi(input[2]);
                motor_id = std::stoi(input[3]);
                break;
            default:
                std::cout << "Command format error\n" <<
                          "\tShould be \"MotorPositionSet <SlaveId> <PassAge> <MotorId> <Position>(0) <Speed>(50) <Current>(500) <AckStatus>(2)\"\n";
                return 1;
        }
    } catch (const std::exception& e) {
        std::cout << "Parameter error" << e.what() << "\n";
        return 1;
    }

    EtherCAT_Msg_ptr msg = std::make_shared<EtherCAT_Msg>();
    set_motor_position(msg.get(), passage, motor_id, pos, spd, cur, ack_status);
    Queue_Msg_ptr queue_msg = createQueueMsg(msg, passage);
    sendToQueue(slaveId, queue_msg);
    return 0;
}

unsigned motorStop(const std::vector<std::string> & input) {
    int slaveId;
    int motor_id;
    int passage;
    try {
        switch (input.size() - 1) {
            case 3:
                slaveId = std::stoi(input[1]);
                passage = std::stoi(input[2]);
                motor_id = std::stoi(input[3]);
                break;
            default:
                std::cout << "Command format error\n" <<
                          "\tShould be \"MotorStop <SlaveId> <PassAge> <MotorId>\"\n";
                return 1;
        }
    } catch (const std::exception& e) {
        std::cout << "Parameter error" << e.what() << "\n";
        return 1;
    }
    EtherCAT_Msg_ptr msg = std::make_shared<EtherCAT_Msg>();
    set_motor_cur_tor(msg.get(), passage, motor_id, 10, 2, 0);
    Queue_Msg_ptr queue_msg = createQueueMsg(msg, passage);
    sendToQueue(slaveId, queue_msg);
    return 0;
}

