#pragma once

typedef struct _LIST_T {
	void *value;
	struct _LIST_T *next;
} LIST_T;

#define LIST_NEW(pp, v) \
	do { \
		LIST_T *tmp = (*pp); \
		(*pp) = malloc(sizeof(LIST_T)); \
		memset((*pp), 0, sizeof(LIST_T)); \
		(*pp)->value = (void*)v; \
		(*pp)->next = tmp; \
	} while(0)

#define LIST_DELETE(pp) \
	do { \
		LIST_T *tmp = (*pp)->next; \
		free(*pp); \
		(*pp) = tmp; \
	} while(0)

#define LIST_TAIL(pp, head) \
	do { \
		for (pp = &head;(*pp) != NULL;pp = &(*pp)->next) {} \
	} while(0)

#define LIST_FOR_START(pp, type, name, head) \
	for (pp = &head;(*pp) != NULL;pp = &(*pp)->next) { \
		type* name = (type*)(*pp)->value;

