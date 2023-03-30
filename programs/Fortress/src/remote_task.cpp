#include "remote_task.h"

remote::DBUS* dbus = nullptr;

void init_dbus(){
    dbus = new remote::DBUS(&huart3);
}