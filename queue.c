#include    <linux/kernel.h>
#include    <linux/slab.h>
#include    <linux/mutex.h>
#include    "queue.h"

typedef struct _queue   Queue;
struct _queue{
    spinlock_t  spin; 
    Element     *first;
    Element     *tail;
};

void *newQueue(void){
    Queue   *q;
    q = kmalloc(sizeof(*q),GFP_ATOMIC);
    if(q){
        spin_lock_init(&q->spin);
        q->first = NULL;
        q->tail  = NULL;
    }
    return q;
}

int enQueue(void *_,Element *e){
    Queue   *q = _;
    if(!q || !e) return -EINVAL;
    spin_lock(&q->spin);
    e->next = NULL;
    if(q->tail) 
        q->tail->next = e;
    else
        q->first = e;
    q->tail = e;
    spin_unlock(&q->spin);
    return 0;
} 

Element *deQueue(void *_){
    Queue   *q = _;
    Element    *e = NULL;
    if(!q)   return NULL;
    spin_lock(&q->spin);
    if(q->first){
        e = q->first;
        q->first = e->next;
        if(!(q->first)){
            q->tail = NULL;
        }
    }
    spin_unlock(&q->spin);
    return e;
}

void delQueue(void *_,void (*fn)(const void *)){
    Queue   *q = _;
    Element *e;
    if(!q) return;
    while(q->first){
        e = q->first;
        q->first = q->first->next;
        fn(e);
    }
}
