#pragma once
#include "main.h"
#include "cmsis_os2.h"
#include "dbus.h"

extern remote::DBUS* dbus;

void init_dbus();