/************************************************************
 * <bsn.cl fy=2016 v=onl>
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
#include <onlp/platformi/sysi.h>
#include <onlp/platformi/ledi.h>
#include <onlp/platformi/fani.h>
#include <onlp/platformi/psui.h>
#include <onlp/platformi/thermali.h>
#include <linux/limits.h>
#include <onlplib/file.h>
#include "arm64_delta_ccba72_log.h"
#include "platform_lib.h"

const char*
onlp_sysi_platform_get(void)
{
    gPlat_id = get_platform_id();

    if (gPlat_id == PID_CCBA72)
        return "arm64-delta-ccba72-r0";
    else
        return "Unknown Platform";
}

int
onlp_sysi_platform_set(const char* platform)
{
    gPlat_id = get_platform_id();

    if (strstr(platform, "arm64-delta-ccba72-r0")) {
        return ONLP_STATUS_OK;
    }

    AIM_LOG_ERROR("Not supported platform '%s'\n", platform);

    return ONLP_STATUS_E_UNSUPPORTED;
}

int
onlp_sysi_platform_info_get(onlp_platform_info_t* pi)
{
    int rc = ONLP_STATUS_OK;
    int len;
    char cpld_ver[4], hw_ver[4];
    char fullpath[PATH_MAX] = {0};
    plat_info_t *plat_info = &gPlat_info[gPlat_id];

    /* CPLD Version */
    sprintf(fullpath, "%s/cpld_version", plat_info->cpld_path);
    if ((rc = plat_os_file_read_str(cpld_ver, sizeof(cpld_ver),
                                     &len, fullpath)) < 0)
        return rc;

    pi->cpld_versions = aim_fstrdup("CPLD Version = %s", cpld_ver);

    /* Hardware Version */
    sprintf(fullpath, "%s/hw_version", plat_info->cpld_path);
    if ((rc = plat_os_file_read_str(hw_ver, sizeof(hw_ver),
                                     &len, fullpath)) < 0)
        return rc;

    pi->other_versions = aim_fstrdup("Hardware Version = %s", hw_ver);

    return rc;
}

int
onlp_sysi_init(void)
{
    return ONLP_STATUS_OK;
}

int
onlp_sysi_onie_data_get(uint8_t** data, int* size)
{
    uint8_t* rdata = aim_zmalloc(256);
    plat_info_t *plat_info = &gPlat_info[gPlat_id];

    if(!rdata){
        return ONLP_STATUS_E_INTERNAL;
    }

    *data = rdata;
    if (onlp_file_read(rdata, 256, size, plat_info->onie_eeprom_path)
        == ONLP_STATUS_OK) {
        if (*size == 256) {
            *data = rdata;
            return ONLP_STATUS_OK;
        }
    }

    aim_free(rdata);
    *size = 0;
    return ONLP_STATUS_E_INTERNAL;
}

void
onlp_sysi_onie_data_free(uint8_t* data)
{
    aim_free(data);
}

int
onlp_sysi_oids_get(onlp_oid_t* table, int max)
{
    int i;
    onlp_oid_t* e = table;
    plat_info_t *plat_info = &gPlat_info[gPlat_id];

    memset(table, 0, max*sizeof(onlp_oid_t));

    /* Thermal sensors on the platform */
    for (i = 1; i <= plat_info->thermal_count; i++) {
        *e++ = ONLP_THERMAL_ID_CREATE(i);
    }

    /* LEDs on the platform */
    for (i = 1; i <= plat_info->led_count; i++) {
        *e++ = ONLP_LED_ID_CREATE(i);
    }

    /* Fans on the platform */
    for (i = 1; i <= plat_info->fan_count; i++) {
        *e++ = ONLP_FAN_ID_CREATE(i);
    }

    /* PSUs on the platform */
    for (i = 1; i <= plat_info->psu_count; i++) {
        *e++ = ONLP_PSU_ID_CREATE(i);
    }

    return 0;
}
