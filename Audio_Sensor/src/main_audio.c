/*sdk verison 2.5.1*/
#include "headers/main_aws.h"
#include "headers/main_audio.h"
#include "headers/main_gsm.h"
#include "headers/main_epoch.h"
#include <zephyr/devicetree.h>
#include <time.h>
#include <string.h>
#include <math.h>
#include <zephyr/device.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/logging/log.h>
#include <zephyr/fs/fs.h>
#include <ff.h>

// For audio
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/audio/dmic.h>
#include <complex.h>

extern struct k_mutex flash_mutex;

const struct device *flash_dev;
LOG_MODULE_REGISTER(audio_start);

// #define AUDIO_FREQ 16000 // for 16 kHz
#define PI 3.14159265358979323846
#define AUDIO_FREQ 8000 // for 8 kHz
#define CHAN_SIZE 16
#define PCM_BLK_SIZE_MS ((AUDIO_FREQ / 1000) * sizeof(int16_t))
#define NUM_MS 5999
K_MEM_SLAB_DEFINE(rx_mem_slab, PCM_BLK_SIZE_MS, NUM_MS + 2, 1);
K_MEM_SLAB_DEFINE(rx_temp_slab, 4, 4, 4);
void *rx_block[NUM_MS];
size_t rx_size = PCM_BLK_SIZE_MS;
extern int writeIdx;
extern int devId;
#define READ_TIMEOUT_MS 6000
#define TARGET_FREQUENCY 2500
#define HAMMING_WINDOW_BLOCK_SIZE 128
// Goertzel algorithm variables
double magnitude = 0.0, mean = 0.0;
double thresholdMean = 39;
#define AUDIO_FRAGMENT 96000

#define SPI_FLASH_SECTOR_SIZE 0x1000
#define READ_ADDRESS 0x1000
#define WRITE_ADDRESS 0x2000
extern int maxWriteIdx;

static FATFS fat_fs;
/* mounting info */
static struct fs_mount_t mp = {
	.type = FS_FATFS,
	.fs_data = &fat_fs,
};

static const char *disk_mount_pt = "/SD:";

static float hamming_window(size_t i)
{
	const float alpha = 0.53836;
	const float beta = 1.0 - alpha;
	float h = (alpha - beta * cosf((2 * PI * i) / (HAMMING_WINDOW_BLOCK_SIZE - 1)));
	return h;
}

static float goertzel(const int16_t *pcm_data[])
{
	printk("Entering goertzel function\n");
	const float sampling_rate = (float)AUDIO_FREQ;
	const float omega = (2.0 * PI * TARGET_FREQUENCY) / sampling_rate;

	// for current, previous and previous than previous value of q
	float q0 = 0.0, q1 = 0.0, q2 = 0.0;
	float cosine = cosf(omega);

	for (size_t i = 0; i < NUM_MS; i++)
	{
		float h = 0.0;
		// Goertzel
		q1 = 0.0;
		q2 = 0.0;
		int16_t data = 0;
		for (size_t j = i; j < i + HAMMING_WINDOW_BLOCK_SIZE / 8; j++)
		{
			for (size_t k = 0; k < 8; k++)
			{
				h = hamming_window((j - i) * 8 + k + 1);
				// Extracting data from audio block
				data = pcm_data[j][k];
				q0 = h * (float)data + 2.0 * cosine * q1 - q2;
				q2 = q1;
				q1 = q0;
				// printk("q0:%f, q1:%f, q2:%f, Data:%d\n", q0, q1, q2, data);
			}
		}
		// Magnitude calculation
		// magnitude = sqrtf(q0 * q0 + q1 * q1 - 2.0 * q0 * q1 * cosine);
		magnitude = cabsf(q1 - cexp(-I * 2 * PI * TARGET_FREQUENCY / sampling_rate) * q2);
		mean += magnitude;
		// printk("Magnittude %i is %f\n", (i / (16)), 20 * log10(magnitude));
		i += 15;
	}
	mean = 20 * log10(mean / 375);
	printk("Mean is %f\n", mean);
	k_sleep(K_MSEC(100));
	return mean;
}

void audio_start(void)
{
	int rc;
	const struct device *flash_dev;
	flash_dev = DEVICE_DT_GET(DT_ALIAS(spi_flash0));
	
	int i;
	uint32_t ms;
	int ret;

	const struct device *const mic_dev = DEVICE_DT_GET(DT_NODELABEL(dmic_dev));

	if (!device_is_ready(mic_dev))
	{
		LOG_ERR("%s: device not ready.\n", mic_dev->name);
		return 0;
	}

	struct pcm_stream_cfg stream = {
		.pcm_width = CHAN_SIZE,
		.mem_slab = &rx_mem_slab,
	};

	struct dmic_cfg cfg = {
		.io = {
			.min_pdm_clk_freq = 512000, // For 8kHz = 64(decimation ratio)*8
			.max_pdm_clk_freq = 640000, // For 8kHz = 80(decimation ratio)*8
			.min_pdm_clk_dc = 40,
			.max_pdm_clk_dc = 60,
		},
		.streams = &stream,
		.channel = {
			.req_num_streams = 1,
		},
	};

	cfg.channel.req_num_chan = 1;
	cfg.channel.req_chan_map_lo =
		dmic_build_channel_map(0, 0, PDM_CHAN_LEFT);
	cfg.streams[0].pcm_rate = AUDIO_FREQ;
	cfg.streams[0].block_size = PCM_BLK_SIZE_MS;

	ret = dmic_configure(mic_dev, &cfg);
	if (ret < 0)
	{
		LOG_ERR("microphone configuration error\n");
		return 0;
	}
	else
	{
		LOG_INF("microphone configuration success\n");
	}

	for (;;)
	{
		int res;
		int epoch_array[10] = {0};
		uint8_t audio_info_part[16] = {0};
		// Audio get data ------------------
		ret = dmic_trigger(mic_dev, DMIC_TRIGGER_START);
		if (ret < 0)
		{
			LOG_ERR("microphone start trigger error\n");
			return 0;
		}
		else
		{
			LOG_INF("microphone start trigger success\n");
		}
		/* Acquire microphone audio */
		printf("Recording...\n");
		for (ms = 0; ms < NUM_MS; ms++)
		{
			ret = dmic_read(mic_dev, 0, &rx_block[ms], &rx_size, READ_TIMEOUT_MS);
			if (ret < 0)
			{
				LOG_ERR("%d microphone audio read error %p %u.\n", ms, rx_block[ms], rx_size);
				return 0;
			}
		}
		float mean = goertzel((const int16_t *)rx_block);
		if (mean > thresholdMean)
		{
			printk("Attempting to write to file...\n");
			char filename[30];

			struct fs_file_t filep;
			size_t bytes_written;

			fs_file_t_init(&filep);

			sprintf(&filename, "/SD:/%ld.raw", writeIdx);

			fs_unlink(filename);

			res = fs_open(&filep, filename, FS_O_CREATE | FS_O_WRITE | FS_O_APPEND);
			if (res)
			{
				printk("Error opening file %s [%d]\n", filename, res);
				// return;
			}
			else
			{
				printk("File %s opened successfully\n", filename);
				setting_epoch();
				get_epoch(epoch_array);
				audio_info_part[0] = devId;
				for (int epoch_count = 0; epoch_count < 10; epoch_count++)
				{
					audio_info_part[epoch_count + 1] = epoch_array[epoch_count];
				}
				fs_write(&filep, &audio_info_part, sizeof(audio_info_part));
			}

			for (i = 0; i < NUM_MS; i++)
			{
				uint32_t *pcm_out = rx_block[i];
				for (int j = 0; j < rx_size / 4; j++)
				{
					// printk("Writing 0x%8x, \n", pcm_out[j]);

					res = fs_write(&filep, &pcm_out[j], sizeof(uint32_t));
					if (res < 0)
					{
						printk("Error writing file [%d]\n", res);
					}
					else
					{
						// printk("%d bytes written to file\n", res);
					}
				}
				k_mem_slab_free(&rx_temp_slab, &pcm_out);
			}
			writeIdx++;
			if (writeIdx > maxWriteIdx)
			{
				writeIdx = 0;
			}

			k_mutex_lock(&flash_mutex, K_FOREVER);
			rc = flash_erase(flash_dev, WRITE_ADDRESS, SPI_FLASH_SECTOR_SIZE);
			if (rc != 0)
			{
				printk("Flash erase failed with error code %d\n", rc);
				return;
			}
			const int wr_len = sizeof(int);
			rc = flash_write(flash_dev, WRITE_ADDRESS, &writeIdx, wr_len);
			if (rc == 0)
			{
				printk("Updated writeIdx : %d\n", writeIdx);
			}
			else
				printk("Failed to update writeIdx\n");
			k_mutex_unlock(&flash_mutex);

			printk("Done Writing\n");
			printk("Done recording\n");

			res = fs_close(&filep);
			if (res < 0)
			{
				printk("%d %d", res, -ENOTSUP);
				printk("Error closing file [%d]\n", res);
				// lsdir(disk_mount_pt);
			}
			for (int b = 0; b < NUM_MS; b++)
				k_mem_slab_free(&rx_mem_slab, &rx_block[b]);
			break;
		}
		else
		{
			printk("Done recording\n");
			printk("Mean is too less to write file in card!!!\n");

			// Freeing memory slab buffer
			for (int b = 0; b < NUM_MS; b++)
				k_mem_slab_free(&rx_mem_slab, &rx_block[b]);
		}
	}
}

void Audio_init()
{

	static const char *disk_pdrv = "SD";
	uint64_t memory_size_mb;
	uint32_t block_count;
	uint32_t block_size;

	if (disk_access_init(disk_pdrv) != 0)
	{
		LOG_ERR("Storage init ERROR!");
		return;
	}

	if (disk_access_ioctl(disk_pdrv,
						  DISK_IOCTL_GET_SECTOR_COUNT, &block_count))
	{
		LOG_ERR("Unable to get sector count");
		return;
	}
	LOG_INF("Block count %u", block_count);

	if (disk_access_ioctl(disk_pdrv,
						  DISK_IOCTL_GET_SECTOR_SIZE, &block_size))
	{
		LOG_ERR("Unable to get sector size");
		return;
	}
	printk("Sector size %u\n", block_size);

	memory_size_mb = (uint64_t)block_count * block_size;
	printk("Memory Size(MB) %u\n", (uint32_t)(memory_size_mb >> 20));
	// }

	mp.mnt_point = disk_mount_pt;

	int res = fs_mount(&mp);

	if (res == FR_OK)
	{
		printk("Disk mounted.\n");
	}
	else
	{
		printk("Error mounting disk.\n");
	}
}