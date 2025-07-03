#include "event/event.h"

#define MAX_EVENT_TYPES      8
#define MAX_HANDLERS_PER_TYPE 4

typedef struct {
    event_type_t type;
    event_handler_t handlers[MAX_HANDLERS_PER_TYPE];
    int count;
} event_subscribers_t;

static event_subscribers_t subscribers[MAX_EVENT_TYPES];

void event_bus_subscribe(event_type_t type, event_handler_t handler) {
    for (int i = 0; i < MAX_EVENT_TYPES; i++) {
        if (subscribers[i].type == type || subscribers[i].count == 0) {
            if (subscribers[i].count == 0) {
                subscribers[i].type = type;
            }

            if (subscribers[i].count >= MAX_HANDLERS_PER_TYPE) {
                return;
            }

            subscribers[i].handlers[subscribers[i].count++] = handler;
            return;
        }
    }
}

void event_bus_publish(event_t *event) {
    for (int i = 0; i < MAX_EVENT_TYPES; i++) {
        if (subscribers[i].count > 0 && subscribers[i].type == event->type) {
            for (int j = 0; j < subscribers[i].count; j++) {
                if (subscribers[i].handlers[j]) {
                    subscribers[i].handlers[j](event);
                }
            }
            return;
        }
    }
}
