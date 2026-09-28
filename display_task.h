/*
 * This file is part of AtomGL.
 *
 * Copyright 2026 Davide Bettio <davide@uninstall.it>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _DISPLAY_TASK_H_
#define _DISPLAY_TASK_H_

#include <stdbool.h>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include <context.h>
#include <mailbox.h>

struct DisplayTaskArgs
{
    QueueHandle_t messages_queue;
    // Handles one message. Returns true when the driver keeps the message
    // (and disposes it itself later), false when the task must dispose it.
    bool (*process_message_fn)(Message *message, Context *ctx);
    Context *ctx;
};

NativeHandlerResult display_task_consume_mailbox(Context *ctx);
void display_task_process_messages(void *arg);

#endif
