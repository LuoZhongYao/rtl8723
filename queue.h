#ifndef __QUEUE_H__
#define __QUEUE_H__


typedef struct element Element;

struct element{
    Element *next;
    size_t  length;
    char    packet[0];
};

void *newQueue(void);
int enQueue(void *queue,Element *element);
Element *deQueue(void *queue);
void delQueue(void *queue,void (*fn)(const void*));
#endif
