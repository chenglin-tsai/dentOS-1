/************************************************************
 * <bsn.cl fy=2014 v=onl>
 *
 *       Copyright 2014, 2015 Big Switch Networks, Inc.
 *       Copyright 2017, 2020, 2021 Delta Networks, Inc.
 *
 * Licensed under the Eclipse Public License, Version 1.0 (the
 * "License"); you may not use this file except in compliance
 * with the License. You may obtain a copy of the License at
 *
 *       http://www.eclipse.org/legal/epl-v10.html
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific
 * language governing permissions and limitations under the
 * License.
 *
 * </bsn.cl>
 ************************************************************
 *
 *
 *
 ***********************************************************/
#ifndef __PLATFORM_LIB_H__
#define __PLATFORM_LIB_H__

#include <onlplib/file.h>
#include <onlplib/i2c.h>

typedef int (*hook_present)(void *e);
typedef int (*hook_event  )(void *e, int ev);

////////////////////////////////////////////////////////////////
// PLATFORM PLAT ROUTINE
typedef enum plat_id {
    PID_CCBA72 = 0x0A,
    PID_UNKNOWN,
} plat_id_t;

typedef struct plat_info {
    char *name;

    char *onie_eeprom_path;

    int cpld_bus;
    char *cpld_path;

    int thermal_count;
    int fan_count;
    int psu_count;
    int led_count;
} plat_info_t;

extern plat_id_t gPlat_id;
extern plat_info_t gPlat_info[];

plat_id_t get_platform_id(void);

////////////////////////////////////////////////////////////////
// THERMAL PLAT ROUTINE
typedef enum plat_thermal_id {
    PLAT_THERMAL_ID_INVALID,
    PLAT_THERMAL_ID_1 = 1,
    PLAT_THERMAL_ID_2,
    PLAT_THERMAL_ID_3,
    PLAT_THERMAL_ID_4,
    PLAT_THERMAL_ID_5,
    PLAT_THERMAL_ID_6,
    PLAT_THERMAL_ID_7,
    PLAT_THERMAL_ID_8,
    PLAT_THERMAL_ID_MAX
} plat_thermal_id_t;

typedef struct plat_thermal {
    char *name;

    hook_present present;
    void *present_data;

    char *temp_get_path;

    char *warning_get_path;
    int  def_warning;

    char *critical_get_path;
    int  def_critical;

    char *shutdown_get_path;
    int  def_shutdown;

    bool enable_set_thresholds;
} plat_thermal_t;

////////////////////////////////////////////////////////////////
// PSU PLAT ROUTINE
typedef enum plat_psu_id {
    PLAT_PSU_ID_INVALID = 0,
    PLAT_PSU_ID_1,
    PLAT_PSU_ID_MAX
} plat_psu_id_t;

typedef enum plat_psu_type {
    PLAT_PSU_TYPE_AC = 0,
    PLAT_PSU_TYPE_DC12,
    PLAT_PSU_TYPE_DC48,
    PLAT_PSU_TYPE_MAX
} plat_psu_type_t;

typedef enum plat_psu_state {
    PLAT_PSU_STATE_PRESENT = 0,
    PLAT_PSU_STATE_UNPRESENT,
    PLAT_PSU_STATE_PMBUS_READY,
    PLAT_PSU_STATE_MAX
} plat_psu_state_t;

typedef enum plat_psu_event {
    PLAT_PSU_EVENT_UNPLUG = 0,
    PLAT_PSU_EVENT_PLUGIN,
    PLAT_PSU_PMBUS_CONNECT,
    PLAT_PSU_PMBUS_DISCONNECT,
    PLAT_PSU_EVENT_MAX
} plat_psu_event_t;

typedef struct plat_psu {
    char *name;

    plat_psu_type_t type;

    hook_present present;
    char *present_path;

    char *power_status_path;

    char *vin_path;
    char *vout_path;
    char *iin_path;
    char *iout_path;
    char *pin_path;
    char *pout_path;

    char *vin_max_path;
    char *vin_min_path;

    char *model_path;
    char *serial_path;

    hook_event event_callback;

    char eeprom_bus;
    char eeprom_addr;

    // use for probe and insmod
    plat_psu_state_t state;
    char *pmbus_insert_cmd;
    char *pmbus_remove_cmd;
    char *pmbus_ready_path;
    uint8_t pmbus_bus;
    uint8_t pmbus_addr;

    uint8_t eeprom[256];
} plat_psu_t;

////////////////////////////////////////////////////////////////
// LED PLAT ROUTINE
typedef enum plat_led_id {
    PLAT_LED_ID_INVALID = 0,
    PLAT_LED_ID_1,
    PLAT_LED_ID_MAX
} plat_led_id_t ;

/* LED related data */
typedef enum led_light_mode {
    LED_MODE_RGB = 0,
    LED_MODE_GB,
    LED_MODE_RB,
    LED_MODE_BLUE,
    LED_MODE_RG,
    LED_MODE_GREEN,
    LED_MODE_RED,
    LED_MODE_OFF,
    LED_MODE_MAX
} led_light_mode_t;

typedef struct led_mode {
    int onlp_led_mode;
    int sys_led_mode;
} led_mode_t;

#define PLAT_LED_MODE(o,s)  { .onlp_led_mode = o, .sys_led_mode = s, }
#define PLAT_LED_MODE_END   { .onlp_led_mode = 1, .sys_led_mode = 1, }

#define PLAT_LED_INTERNAL_DEF \
    .cur_led_mode = -1

typedef struct plat_led {
    char *name;
    char *led_get_path;
    char *led_set_path;
    led_mode_t mode[LED_MODE_MAX];
    int cur_led_mode;
} plat_led_t;

////////////////////////////////////////////////////////////////
// OS HELP ROUTINE
extern int plat_os_file_is_existed(char *path);
extern int plat_os_file_read(uint8_t *data, int max, int *len, char *path, ...);
extern int plat_os_file_read_str(char *data, int max, int *len, char *path, ...);
extern int plat_os_file_read_int(int *val, char *path, ...);
extern int plat_os_file_write_int(int  val, char *path, ...);

int eeprom_read (int bus, uint8_t addr, uint8_t offset, uint8_t *buff, int len);

#endif  /* __PLATFORM_LIB_H__ */

