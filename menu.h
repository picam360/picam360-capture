#ifndef _MREVENT_H
#define _MREVENT_H

#include <pthread.h>
#include <stdbool.h>

enum MENU_EVENT{
	NONE,
	ACTIVATED,
	DEACTIVATED,
	SELECTED,
	DESELECTED,
};
enum MENU_OPERATE{
	NONE,
	ACTIVE_BACK,
	ACTIVE_NEXT,
	SELECT,
	DESELECT,
};

typedef struct _MENU;
typedef void (*MENU_CALLBACK)(struct _MENU *menu, MENU_EVENT event);
typedef struct _MENU{
	char name[256];
	bool activated;
	MENU_CALLBACK callback;
	struct _MENU *submenu[256];
} MENU_T;


void init_menu();
void deinit_menu();
void menu_redraw(MENU_T *root);
MENU_T *menu_new(char *name, MENU_CALLBACK callback);
void menu_delete(MENU_T **menu);
void menu_add_submenu(MENU_T *parent, MENU_T *child, int idx);
void menu_operate(MENU_T *menu, MENU_OPERATE operate);

#endif
