#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/io.h>
#include <stdio.h>
#include <errno.h>

#include "legato.h"
#include "interfaces.h"
#include "le_mutex.h"
#include "json.h"

#define TEMPERATURE_DATAHUB	"infraredTemperature/pixel"

static void json_extract_dump(le_result_t res)
{
	if (res == LE_OK) {
		LE_INFO("json_Extract: successful");
	}
	if (res == LE_FORMAT_ERROR) {
		LE_ERROR("json_Extract: there's something wrong with the input JSON string.");
	}
	if (res == LE_BAD_PARAMETER) {
		LE_ERROR("json_Extract: there's something wrong with the extraction specification");
	}
	if (res == LE_NOT_FOUND) {
		LE_ERROR("json_Extract: the thing we are trying to extract doesn't exist in the JSON input");
	}
	if (res == LE_OVERFLOW) {
		LE_ERROR("json_Extract: the provided result buffer isn't big enough");
	}
}

static void TempeartureDataHubHandle(double timestamp,
				  const char* LE_NONNULL value,
				  void* contextPtr)
{
	char buffer[IO_MAX_STRING_VALUE_LEN];
	double pixels[64];
	int index;
	double temp_value;
	le_result_t le_res;
	json_DataType_t json_data_type;

	LE_INFO("TemperatureDataHubHandle: timestamp %lf", timestamp);
	LE_INFO("TemperatureDataHubHandle: value %s", value);

	if (!json_IsValid(value)) {
		LE_ERROR("INVALID JSON string");
		return;
	}
	 
	memset(buffer, 0, IO_MAX_STRING_VALUE_LEN);

	le_res = json_Extract(buffer,
			      IO_MAX_STRING_VALUE_LEN,
			      value,
		              "index", 
			      &json_data_type);
	json_extract_dump(le_res);
	if (json_data_type != JSON_TYPE_NUMBER) {
		LE_ERROR("WRONG data type for index");
		return;
	}
	index = (int)json_ConvertToNumber(buffer);

	le_res = json_Extract(buffer,
			      IO_MAX_STRING_VALUE_LEN,
			      value,
		              "value", 
		              &json_data_type);
	json_extract_dump(le_res);
	if (json_data_type != JSON_TYPE_NUMBER) {
		LE_ERROR("WRONG data type for value");
		return;
	}
	temp_value = (double)json_ConvertToNumber(buffer);

	pixels[index] = temp_value;
	LE_INFO("Tempearture of pixel:%d is %0.2f \n", index, pixels[index]);
}

COMPONENT_INIT
{
	le_result_t result;
	ma_infraredTemperature_Init();

	// This will be received from the Data Hub.
	result = io_CreateOutput(TEMPERATURE_DATAHUB,
				 IO_DATA_TYPE_JSON,
				 "Tempearture");
	LE_ASSERT(result == LE_OK);

	// Register for notification of updates to the counter value.
	io_AddJsonPushHandler(TEMPERATURE_DATAHUB,
			      TempeartureDataHubHandle,
			      NULL);
}
