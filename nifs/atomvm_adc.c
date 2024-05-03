//
// Copyright (c) 2020 dushin.net
// Copyright (c) 2024 Ricardo Lanziano <arpunk@fatelectron.net>
// Copyright (c) 2023 Winford <winford@object.stream>
// All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//

// References
// https://docs.espressif.com/projects/esp-idf/en/v5.1.2/esp32/api-reference/peripherals/adc_oneshot.html
// https://docs.espressif.com/projects/esp-idf/en/v5.1.2/esp32/api-reference/peripherals/adc_calibration.html
//

#include "atomvm_adc.h"

#include <context.h>
#include <defaultatoms.h>
#include <erl_nif_priv.h>
#include <globalcontext.h>
#include <interop.h>
#include <module.h>
#include <nifs.h>
#include <term.h>

// #define ENABLE_TRACE
#include <trace.h>

#include <esp_adc/adc_oneshot.h>
#include <esp32_sys.h>
#include <esp_adc/adc_cali.h>
#include <esp_log.h>
#include <sdkconfig.h>

#include <stdlib.h>

#define TAG "atomvm_adc"
#define DEFAULT_SAMPLES 64
#define DEFAULT_VREF 1100

#define AVM_ADC_ATTEN_INVALID 0xFF

static ErlNifResourceType *adc_resource_type;

struct ADCResource
{
    term transmitting_pid;
    adc_channel_t channel;
    adc_oneshot_unit_handle_t adc1_handle;
    adc_cali_handle_t adc1_cali_chan0_handle;
    bool cal_1_chan_0;
    adc_cali_handle_t adc2_cali_handler;
    adc_oneshot_unit_handle_t adc2_handle;
    bool cal_2;
};

#ifdef CONFIG_AVM_ADC2_ENABLE
static const char *const timeout_atom = ATOM_STR("\x7", "timeout");
#endif

static const char *const invalid_adc_read_atom = ATOM_STR("\xc", "invalid_read");
static const char *const calibration_error_atom = ATOM_STR("\x11", "calibration_error");

//ADC1 Channels
#if CONFIG_IDF_TARGET_ESP32
#define ATOMVM_ADC1_CHAN0          ADC_CHANNEL_4
#define ATOMVM_ADC1_CHAN1          ADC_CHANNEL_5
#else
#define ATOMVM_ADC1_CHAN0          ADC_CHANNEL_0
#define ATOMVM_ADC1_CHAN1          ADC_CHANNEL_1
#endif

#if (SOC_ADC_PERIPH_NUM >= 2) && !CONFIG_IDF_TARGET_ESP32C3
/**
 * On ESP32C3, ADC2 is no longer supported, due to its HW limitation.
 * Search for errata on espressif website for more details.
 */
#define ATOMVM_ADC_USE_ADC2            1
#endif

#if ATOMVM_USE_ADC2
//ADC2 Channels
#if CONFIG_IDF_TARGET_ESP32
#define ATOMVM_ADC2_CHAN0          ADC_CHANNEL_0
#else
#define ATOMVM_ADC2_CHAN0          ADC_CHANNEL_0
#endif
#endif  //#if ATOMVM_USE_ADC2

static const AtomStringIntPair bit_width_table[] = {
    { ATOM_STR("\x7", "bit_max"), (ADC_BITWIDTH_DEFAULT) },
#if SOC_ADC_MAX_BITWIDTH == 13
    { ATOM_STR("\x6", "bit_13"), ADC_BITWIDTH_13 },
#elif SOC_ADC_MAX_BITWIDTH == 12
    { ATOM_STR("\x6", "bit_12"), ADC_BITWIDTH_12 },
#elif CONFIG_IDF_TARGET_ESP32
    { ATOM_STR("\x6", "bit_11"), ADC_BITWIDTH_11 },
    { ATOM_STR("\x6", "bit_10"), ADC_BITWIDTH_10 },
    { ATOM_STR("\x5", "bit_9"), ADC_BITWIDTH_9 },
#endif
    SELECT_INT_DEFAULT(ADC_BITWIDTH_DEFAULT)
};

static const AtomStringIntPair attenuation_table[] = {
    { ATOM_STR("\x4", "db_0"), ADC_ATTEN_DB_0 },
    { ATOM_STR("\x6", "db_2_5"), ADC_ATTEN_DB_2_5 },
    { ATOM_STR("\x4", "db_6"), ADC_ATTEN_DB_6 },
    { ATOM_STR("\x5", "db_11"), ADC_ATTEN_DB_11 },
#ifdef ADC_ATTEN_DB_12
    { ATOM_STR("\x5", "db_12"), ADC_ATTEN_DB_12 },
#endif
    SELECT_INT_DEFAULT(AVM_ADC_ATTEN_INVALID)
};

static adc_unit_t adc_unit_from_pin(int pin_val)
{
    switch (pin_val) {
#if CONFIG_IDF_TARGET_ESP32
        case 32:
        case 33:
        case 34:
        case 35:
        case 36:
        case 37:
        case 38:
        case 39:
#elif CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3
        case 1:
        case 2:
        case 3:
        case 4:
        case 5:
        case 6:
        case 7:
        case 8:
        case 9:
        case 10:
#elif CONFIG_IDF_TARGET_ESP32C3
        case 0:
        case 1:
        case 2:
        case 3:
        case 4:
#endif
            return ADC_UNIT_1;
#ifdef CONFIG_AVM_ADC2_ENABLE
#if CONFIG_IDF_TARGET_ESP32
        case 0:
        case 2:
        case 4:
        case 12:
        case 13:
        case 14:
        case 15:
        case 25:
        case 26:
        case 27:
#elif CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32S2
        case 11:
        case 12:
        case 13:
        case 14:
        case 15:
        case 16:
        case 17:
        case 18:
        case 19:
        case 20:
#elif CONFIG_IDF_TARGET_ESP32C3
        case 5:
#endif
            return ADC_UNIT_2;
#endif
        default:
            return SOC_ADC_PERIPH_NUM;
    }
}

static adc_channel_t get_channel(avm_int_t pin_val)
{
    switch (pin_val) {
#if CONFIG_IDF_TARGET_ESP32
        case 32:
            return ADC_CHANNEL_4;
        case 33:
            return ADC_CHANNEL_5;
        case 34:
            return ADC_CHANNEL_6;
        case 35:
            return ADC_CHANNEL_7;
        case 36:
            return ADC_CHANNEL_0;
        case 37:
            return ADC_CHANNEL_1;
        case 38:
            return ADC_CHANNEL_2;
        case 39:
            return ADC_CHANNEL_3;
#elif CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3
        case 1:
            return ADC_CHANNEL_0;
        case 2:
            return ADC_CHANNEL_1;
        case 3:
            return ADC_CHANNEL_2;
        case 4:
            return ADC_CHANNEL_3;
        case 5:
            return ADC_CHANNEL_4;
        case 6:
            return ADC_CHANNEL_5;
        case 7:
            return ADC_CHANNEL_6;
        case 8:
            return ADC_CHANNEL_7;
        case 9:
            return ADC_CHANNEL_8;
        case 10:
            return ADC_CHANNEL_9;
#elif CONFIG_IDF_TARGET_ESP32C3
        case 0:
            return ADC_CHANNEL_0;
        case 1:
            return ADC_CHANNEL_1;
        case 2:
            return ADC_CHANNEL_2;
        case 3:
            return ADC_CHANNEL_3;
        case 4:
            return ADC_CHANNEL_4;
#endif
#ifdef CONFIG_AVM_ADC2_ENABLE
#if CONFIG_IDF_TARGET_ESP32
        case 0:
            return ADC_CHANNEL_1;
        case 2:
            return ADC_CHANNEL_2;
        case 4:
            return ADC_CHANNEL_0;
        case 12:
            return ADC_CHANNEL_5;
        case 13:
            return ADC_CHANNEL_4;
        case 14:
            return ADC_CHANNEL_6;
        case 15:
            return ADC_CHANNEL_3;
        case 25:
            return ADC_CHANNEL_8;
        case 26:
            return ADC_CHANNEL_9;
        case 27:
            return ADC_CHANNEL_7;
#elif CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3
        case 11:
            return ADC_CHANNEL_0;
        case 12:
            return ADC_CHANNEL_1;
        case 13:
            return ADC_CHANNEL_2;
        case 14:
            return ADC_CHANNEL_3;
        case 15:
            return ADC_CHANNEL_4;
        case 16:
            return ADC_CHANNEL_5;
        case 17:
            return ADC_CHANNEL_6;
        case 18:
            return ADC_CHANNEL_7;
        case 19:
            return ADC_CHANNEL_8;
        case 20:
            return ADC_CHANNEL_9;
#elif CONFIG_IDF_TARGET_ESP32C3
        case 5:
            return ADC_CHANNEL_0;
#endif
#endif
        default:
            return ADC_CHANNEL_0;
    }
}

// Wrapper function for line/curve fitting calibration
static bool do_calibration(adc_unit_t unit,
			   adc_channel_t channel,
			   adc_bitwidth_t bit_width,
			   adc_atten_t atten,
			   adc_cali_handle_t *handle)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
  UNUSED(channel);

  adc_cali_curve_fitting_config_t cali_config = {
    .unit_id = unit,
    .chan = channel,
    .atten = atten,
    .bitwidth = bit_width,
  };
  ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
  return adc_cali_create_scheme_curve_fitting(&cali_config, handle) == ESP_OK;
#endif
#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
  adc_cali_line_fitting_config_ cali_config = {
    .unit_id = unit,
    .atten = atten,
    .bitwidth = bit_width,
  };
  ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
  return adc_cali_create_scheme_line_fitting(&cali_config, handle) == ESP_OK;
#endif
}

static bool nif_adc_calibration_init(adc_unit_t unit,
				     adc_channel_t channel,
				     adc_bitwidth_t bit_width,
				     adc_atten_t atten,
				     adc_cali_handle_t *out_handle)
{
  bool calibrated = do_calibration(unit, channel, bit_width, atten, out_handle);

  if (calibrated) {
    ESP_LOGI(TAG, "calibration successful");
    TRACE("Attenuation on channel %u set to %u, bit width %u\n", channel, atten, bit_width);
  } else {
    ESP_LOGW(TAG, "calibration failed or not supported.");
  }

  return calibrated;
}

static void nif_adc_calibration_deinit(adc_cali_handle_t handle)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Curve Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));

#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Line Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
#endif
}

static term create_triple(Context *ctx, term term1, term term2, term term3)
{
    term ret = term_alloc_tuple(3, &ctx->heap);
    term_put_tuple_element(ret, 0, term1);
    term_put_tuple_element(ret, 1, term2);
    term_put_tuple_element(ret, 2, term3);

    return ret;
}

static bool init_adc_channel(adc_oneshot_unit_handle_t *handle,
			     adc_channel_t channel,
                             adc_bitwidth_t bit_width,
			     adc_atten_t atten)
{
  adc_cali_handle_t adc_cali_chan_handle = NULL;

  adc_oneshot_unit_init_cfg_t init_config = {
    .unit_id = channel,  // TODO: might be an error?
#if ATOMVM_USE_ADC2
    .ulp_mode = ADC_ULP_MODE_DISABLE,
#endif  //#if ATOMVM_USE_ADC2
  };

  ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, handle));

  adc_oneshot_chan_cfg_t config = {
    .bitwidth = bit_width,
    .atten = atten,
  };

  return ESP_OK == adc_oneshot_config_channel(*handle, channel, &config);
}

static term nif_adc_open(Context *ctx, int argc, term argv[])
{
    UNUSED(argc);
    GlobalContext *global = ctx->global;
    term opts = argv[0];

    term pin = interop_kv_get_value(opts, ATOM_STR("\x3", "pin"), global);
    term width = interop_kv_get_value(opts, ATOM_STR("\x8", "bitwidth"), global);
    term attenuation = interop_kv_get_value(opts, ATOM_STR("\xb", "attenuation"), global);

    VALIDATE_VALUE(pin, term_is_integer);
    VALIDATE_VALUE(attenuation, term_is_atom);
    VALIDATE_VALUE(width, term_is_atom);

    struct ADCResource *rsrc_obj = enif_alloc_resource(adc_resource_type, sizeof(struct ADCResource));
    if (IS_NULL_PTR(rsrc_obj)) {
        ESP_LOGW(TAG, "failed to allocate memory: %s:%i.\n", __FILE__, __LINE__);
        RAISE_ERROR(OUT_OF_MEMORY_ATOM);
    }

    adc_channel_t channel = get_channel(pin);
    adc_bitwidth_t bit_width = interop_atom_term_select_int(bit_width_table, width, global);
    adc_atten_t atten = interop_atom_term_select_int(attenuation_table, attenuation, global);

    rsrc_obj->transmitting_pid = term_invalid_term();
    rsrc_obj->channel = channel;

    // ADC1 init and config
    adc_oneshot_unit_handle_t adc1_handle;
    if (!init_adc_channel(&adc1_handle, channel, bit_width, atten)) {
      enif_release_resource(rsrc_obj);
      ESP_LOGE(TAG, "ADC1 initialization failed");
      RAISE_ERROR(ERROR_ATOM);
    }

    // ADC1 calibration init
    adc_cali_handle_t adc1_cali_chan0_handle = NULL;
    bool do_cal1_chan0 = nif_adc_calibration_init(ADC_UNIT_1, channel, bit_width, atten, &adc1_cali_chan0_handle);

    rsrc_obj->adc1_handle = adc1_handle;
    rsrc_obj->adc1_cali_chan0_handle = adc1_cali_chan0_handle;
    rsrc_obj->cal_1_chan_0 = do_cal1_chan0;

#if ATOMVM_USE_ADC2
    // ADC2 init and config
    adc_oneshot_unit_handle_t adc2_handle;
    if (!init_adc_channel(&adc2_handle, channel, bit_width, atten)) {
      enif_release_resource(rsrc_obj);
      ESP_LOGE(TAG, "ADC2 initialization failed");
      RAISE_ERROR(ERROR_ATOM);
    }

    // ADC2 calibration init
    adc_cali_handle_t adc2_cali_handle = NULL;
    bool do_cal2 = nif_adc_calibration_init(ADC_UNIT_2, channel, bit_width, atten, &adc2_cali_handle);

    rsrc_obj->adc2_cali_handler = adc2_cali_handler;
    rsrc_obj->adc2_handle = adc2_handle;
    rsrc_obj->cal_2 = do_cal2;
#endif  //#if ATOMVM_USE_ADC2

    if (UNLIKELY(memory_ensure_free(ctx, TERM_BOXED_RESOURCE_SIZE) != MEMORY_GC_OK)) {
        enif_release_resource(rsrc_obj);
        ESP_LOGW(TAG, "failed to allocate memory: %s:%i.", __FILE__, __LINE__);
        RAISE_ERROR(OUT_OF_MEMORY_ATOM);
    }

    term obj = enif_make_resource(erl_nif_env_from_context(ctx), rsrc_obj);
    enif_release_resource(rsrc_obj);

    // {'$adc', Resource :: resource(), Ref :: reference()} :: adc()
    size_t requested_size = TUPLE_SIZE(3) + REF_SIZE;
    if (UNLIKELY(memory_ensure_free_with_roots(ctx, requested_size, 1, &obj, MEMORY_CAN_SHRINK) != MEMORY_GC_OK)) {
        ESP_LOGW(TAG, "failed to allocate memory: %s:%i.", __FILE__, __LINE__);
        RAISE_ERROR(OUT_OF_MEMORY_ATOM);
    }

    term adc = term_alloc_tuple(3, &ctx->heap);
    term_put_tuple_element(adc, 0, globalcontext_make_atom(global, ATOM_STR("\x4", "$adc")));
    term_put_tuple_element(adc, 1, obj);
    uint64_t ref_ticks = globalcontext_get_ref_ticks(ctx->global);
    term ref = term_from_ref_ticks(ref_ticks, &ctx->heap);
    term_put_tuple_element(adc, 2, ref);

    return adc;
}

// Copied from I2C resource NIF
static bool is_adc_resource(GlobalContext *global, term t)
{
    bool ret = term_is_tuple(t)
        && term_get_tuple_arity(t) == 3
        && globalcontext_is_term_equal_to_atom_string(global, term_get_tuple_element(t, 0), ATOM_STR("\x4", "$adc"))
        && term_is_binary(term_get_tuple_element(t, 1))
        && term_is_reference(term_get_tuple_element(t, 2));

    return ret;
}

// Copied from I2C resource NIF
static bool to_adc_resource(term adc_resource, struct ADCResource **rsrc_obj, Context *ctx)
{
    if (!is_adc_resource(ctx->global, adc_resource)) {
        return false;
    }
    void *rsrc_obj_ptr;
    if (UNLIKELY(!enif_get_resource(erl_nif_env_from_context(ctx),
				    term_get_tuple_element(adc_resource, 1),
				    adc_resource_type,
				    &rsrc_obj_ptr))) {
        return false;
    }
    *rsrc_obj = (struct ADCResource *) rsrc_obj_ptr;

    return true;
}

static term nif_adc_take_reading(Context *ctx, int argc, term argv[])
{
    UNUSED(argc);

    GlobalContext *global = ctx->global;

    term adc_resource = argv[0];
    struct ADCResource *rsrc_obj;
    if (UNLIKELY(!to_adc_resource(adc_resource, &rsrc_obj, ctx))) {
      ESP_LOGE(TAG, "failed to convert adc_resource");
      RAISE_ERROR(BADARG_ATOM);
    }

    term read_options = argv[1];
    VALIDATE_VALUE(read_options, term_is_list);
    term samples = interop_kv_get_value_default(read_options, ATOM_STR("\x7", "samples"), DEFAULT_SAMPLES, global);

    avm_int_t samples_val = term_to_int(samples);

    if (samples_val <= 0) {
      ESP_LOGE(TAG, "invalid number of samples");
      RAISE_ERROR(BADARG_ATOM);
    }

    term raw = interop_kv_get_value_default(read_options, ATOM_STR("\x3", "raw"), FALSE_ATOM, global);
    term voltage_option = interop_kv_get_value_default(read_options, ATOM_STR("\x7", "voltage"), FALSE_ATOM, global);

    uint32_t v = 0, r = 0;
    int adc_raw = 0, voltage;

    for (avm_int_t i = 0; i < samples_val; ++i) {
      if (ESP_OK != adc_oneshot_read(rsrc_obj->adc1_handle, rsrc_obj->channel, &adc_raw)) {
	ESP_LOGE(TAG, "ADC1 read error");
	RAISE_ERROR(invalid_adc_read_atom);
      }

      r += adc_raw;

      if (voltage_option == TRUE_ATOM && rsrc_obj->cal_1_chan_0) {
	if (ESP_OK != adc_cali_raw_to_voltage(rsrc_obj->adc1_cali_chan0_handle, adc_raw, &voltage)) {
	  ESP_LOGE(TAG, "ADC1 calibration conversion error");
	  RAISE_ERROR(calibration_error_atom);
	}

	v += voltage;
      }
#if ATOMVM_USE_ADC2
      if (ESP_OK != adc_oneshot_read(rsrc_obj->adc2_handle, rsrc_obj->channel, &adc_raw)) {
	ESP_LOGE(TAG, "ADC2 read error");
	RAISE_ERROR(invalid_adc_read_atom);
      }
      if (voltage_option == TRUE_ATOM && rsrc_obj->cal_2) {
	if (ESP_OK != adc_cali_raw_to_voltage(rsrc_obj->adc2_cali_handle, adc_raw, &voltage)) {
	  ESP_LOGE(TAG, "ADC2 calibration conversion error");
	  RAISE_ERROR(calibration_error_atom);
	}

	v += voltage;
      }
#endif  //#if ATOMVM_USE_ADC2
    }

    term final_raw = raw == TRUE_ATOM ? term_from_int32(r / samples_val) : UNDEFINED_ATOM;
    term final_voltage = voltage_option == TRUE_ATOM ? term_from_int32(v / samples_val) : UNDEFINED_ATOM;

    if (UNLIKELY(memory_ensure_free(ctx, TUPLE_SIZE(3)) != MEMORY_GC_OK)) {
      RAISE_ERROR(OUT_OF_MEMORY_ATOM);
    } else {
      return create_triple(ctx, OK_ATOM, final_raw, final_voltage);
    }
}

static void nif_adc_resource_dtor(ErlNifEnv *caller_env, void *obj)
{
    UNUSED(caller_env);
    struct ADCResource *rsrc_obj = (struct ADCResource *) obj;

    esp_err_t err = ESP_OK;
    if (UNLIKELY(err != ESP_OK)) {
        ESP_LOGW(TAG, "failed to delete driver in resource d'tor.  err=%i", err);
    }

    ESP_ERROR_CHECK(adc_oneshot_del_unit(rsrc_obj->adc1_handle));
    if (rsrc_obj->cal_1_chan_0) {
      nif_adc_calibration_deinit(rsrc_obj->adc1_cali_chan0_handle);
    }

#if ATOMVM_USE_ADC2
    ESP_ERROR_CHECK(adc_oneshot_del_unit(rsrc_obj->adc2_handle));
    if (rsrc_obj->cal_2) {
      nif_adc_calibration_deinit(rsrc_obj->adc2_cali_handle);
    }

#endif //#if ATOMVM_USE_ADC2
}

static const ErlNifResourceTypeInit ADCResourceTypeInit = {
    .members = 1,
    .dtor = nif_adc_resource_dtor,
};

static const struct Nif adc_open_nif = {
    .base.type = NIFFunctionType,
    .nif_ptr = nif_adc_open
};
static const struct Nif adc_take_reading_nif = {
    .base.type = NIFFunctionType,
    .nif_ptr = nif_adc_take_reading
};

//
// Component Nif Entrypoints
//

void atomvm_adc_init(GlobalContext *global)
{
    ErlNifEnv env;
    erl_nif_env_partial_init_from_globalcontext(&env, global);
    adc_resource_type = enif_init_resource_type(&env, "adc_resource", &ADCResourceTypeInit, ERL_NIF_RT_CREATE, NULL);
}

const struct Nif *atomvm_adc_get_nif(const char *nifname)
{
    TRACE("Locating nif %s ...", nifname);
    if (strcmp("adc:open_nif/1", nifname) == 0) {
        TRACE("Resolved adc nif %s ...\n", nifname);
        return &adc_open_nif;
    }
    if (strcmp("adc:take_reading_nif/2", nifname) == 0) {
        TRACE("Resolved adc nif %s ... \n", nifname);
	return &adc_take_reading_nif;
    }
    return NULL;
}

#include <sdkconfig.h>

#ifdef CONFIG_AVM_ADC_ENABLE
REGISTER_NIF_COLLECTION(atomvm_adc, atomvm_adc_init, NULL, atomvm_adc_get_nif)
#endif
