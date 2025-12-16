// Copyright 2020, Ryan Wendland, usb64
// SPDX-License-Identifier: MIT

#include "tusb_option.h"

#if (TUSB_OPT_HOST_ENABLED && CFG_TUH_XINPUT)

#include "host/usbh.h"
#include "host/usbh_pvt.h"
#include "xinput_host.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-const-variable"

//Wired 360 commands
static const uint8_t xbox360_wired_rumble[] = {0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static const uint8_t xbox360_wired_led[] = {0x01, 0x03, 0x00};

//Xbone one
#define GIP_CMD_ACK 0x01
#define GIP_CMD_ANNOUNCE 0x02
#define GIP_CMD_IDENTIFY 0x04
#define GIP_CMD_POWER 0x05
#define GIP_CMD_AUTHENTICATE 0x06
#define GIP_CMD_VIRTUAL_KEY 0x07
#define GIP_CMD_RUMBLE 0x09
#define GIP_CMD_LED 0x0a
#define GIP_CMD_FIRMWARE 0x0c
#define GIP_CMD_INPUT 0x20
#define GIP_SEQ0 0x00
#define GIP_OPT_ACK 0x10
#define GIP_OPT_INTERNAL 0x20
#define GIP_PL_LEN(N) (N)
#define GIP_PWR_ON 0x00
#define GIP_PWR_SLEEP 0x01
#define GIP_PWR_OFF 0x04
#define GIP_PWR_RESET 0x07
#define GIP_LED_ON 0x01
#define BIT(n) (1UL << (n))
#define GIP_MOTOR_R  BIT(0)
#define GIP_MOTOR_L  BIT(1)
#define GIP_MOTOR_RT BIT(2)
#define GIP_MOTOR_LT BIT(3)
#define GIP_MOTOR_ALL (GIP_MOTOR_R | GIP_MOTOR_L | GIP_MOTOR_RT | GIP_MOTOR_LT)
static const uint8_t xboxone_power_on[] = {GIP_CMD_POWER, GIP_OPT_INTERNAL, GIP_SEQ0, GIP_PL_LEN(1), GIP_PWR_ON};
static const uint8_t xboxone_s_init[] = {GIP_CMD_POWER, GIP_OPT_INTERNAL, GIP_SEQ0, GIP_PL_LEN(15), 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x55, 0x53, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static const uint8_t xboxone_s_led_init[] = {GIP_CMD_LED, GIP_OPT_INTERNAL, GIP_SEQ0, GIP_PL_LEN(3), 0x00, 0x01, 0x14};
static const uint8_t extra_input_packet_init[] = {0x4d, 0x10, GIP_SEQ0, 0x02, 0x07, 0x00};
static const uint8_t xboxone_pdp_led_on[] = {GIP_CMD_LED, GIP_OPT_INTERNAL, GIP_SEQ0, GIP_PL_LEN(3), 0x00, GIP_LED_ON, 0x14};
static const uint8_t xboxone_pdp_auth[] = {GIP_CMD_AUTHENTICATE, GIP_OPT_INTERNAL, GIP_SEQ0, GIP_PL_LEN(2), 0x01, 0x00};
static const uint8_t xboxone_rumble[] = {GIP_CMD_RUMBLE, 0x00, 0x00, GIP_PL_LEN(9), 0x00, GIP_MOTOR_ALL, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0xFF};

// Wired 360 chatpad control transfer commands
static const tusb_control_request_t xbox360_chatpad_init1 = { .bmRequestType = 0x40, .bRequest = 0xA9, .wValue = 0xA30C, .wIndex = 0x4423, .wLength = 0 };
static const tusb_control_request_t xbox360_chatpad_init2 = { .bmRequestType = 0x40, .bRequest = 0xA9, .wValue = 0x2344, .wIndex = 0x7F03, .wLength = 0 };
static const tusb_control_request_t xbox360_chatpad_init3 = { .bmRequestType = 0x40, .bRequest = 0xA9, .wValue = 0x5839, .wIndex = 0x6832, .wLength = 0 };
static const tusb_control_request_t xbox360_chatpad_init4 = { .bmRequestType = 0xC0, .bRequest = 0xA1, .wValue = 0x0000, .wIndex = 0xE416, .wLength = 2 };
static const tusb_control_request_t xbox360_chatpad_init5 = { .bmRequestType = 0x40, .bRequest = 0xA1, .wValue = 0x0000, .wIndex = 0xE416, .wLength = 2 };
static const tusb_control_request_t xbox360_chatpad_init6 = { .bmRequestType = 0xC0, .bRequest = 0xA1, .wValue = 0x0000, .wIndex = 0xE416, .wLength = 2 };
static const tusb_control_request_t xbox360_chatpad_keepalive1 = { .bmRequestType = 0x41, .bRequest = 0x00, .wValue = 0x001F, .wIndex = 0x0002, .wLength = 0 };
static const tusb_control_request_t xbox360_chatpad_keepalive2 = { .bmRequestType = 0x41, .bRequest = 0x00, .wValue = 0x001E, .wIndex = 0x0002, .wLength = 0 };
static const tusb_control_request_t xbox360_chatpad_leds_on = { .bmRequestType = 0x41, .bRequest = 0x00, .wValue = 0x001B, .wIndex = 0x0002, .wLength = 0 };

//Wireless 360 commands
static const uint8_t xbox360w_led[] = {0x00, 0x00, 0x08, 0x40};
//Sending 0x00, 0x00, 0x08, 0x00 will permanently disable rumble until you do this:
static const uint8_t xbox360w_rumble_enable[] = {0x00, 0x00, 0x08, 0x01};
static const uint8_t xbox360w_rumble[] = {0x00, 0x01, 0x0F, 0xC0, 0x00, 0x00, 0x00};
static const uint8_t xbox360w_inquire_present[] = {0x08, 0x00, 0x0F, 0xC0};
static const uint8_t xbox360w_controller_info[] = {0x00, 0x00, 0x00, 0x40};
static const uint8_t xbox360w_unknown[] = {0x00, 0x00, 0x02, 0x80};
static const uint8_t xbox360w_power_off[] = {0x00, 0x00, 0x08, 0xC0};
static const uint8_t xbox360w_chatpad_init[] = {0x00, 0x00, 0x0C, 0x1B};
static const uint8_t xbox360w_chatpad_keepalive1[] = {0x00, 0x00, 0x0C, 0x1F};
static const uint8_t xbox360w_chatpad_keepalive2[] = {0x00, 0x00, 0x0C, 0x1E};

//Original Xbox
static const uint8_t xboxog_rumble[] = {0x00, 0x06, 0x00, 0x00, 0x00, 0x00};

#pragma GCC diagnostic pop

typedef struct
{
    uint8_t inst_count;
    xinputh_interface_t instances[CFG_TUH_XINPUT];
} xinputh_device_t;

static xinputh_device_t _xinputh_dev[CFG_TUH_DEVICE_MAX];

TU_ATTR_ALWAYS_INLINE static inline xinputh_device_t *get_dev(uint8_t dev_addr)
{
    // Bounds check to prevent crashes
    if (dev_addr == 0 || dev_addr > CFG_TUH_DEVICE_MAX) {
        printf("[XINPUT] ERROR: get_dev out of bounds dev_addr=%d (max=%d)\n", dev_addr, CFG_TUH_DEVICE_MAX);
        return NULL;
    }
    return &_xinputh_dev[dev_addr - 1];
}

TU_ATTR_ALWAYS_INLINE static inline xinputh_interface_t *get_instance(uint8_t dev_addr, uint8_t instance)
{
    // Bounds check to prevent crashes
    if (dev_addr == 0 || dev_addr > CFG_TUH_DEVICE_MAX || instance >= CFG_TUH_XINPUT) {
        printf("[XINPUT] ERROR: get_instance out of bounds dev_addr=%d instance=%d\n", dev_addr, instance);
        return NULL;
    }
    return &_xinputh_dev[dev_addr - 1].instances[instance];
}

static uint8_t get_instance_id_by_epaddr(uint8_t dev_addr, uint8_t ep_addr)
{
    for (uint8_t inst = 0; inst < CFG_TUH_XINPUT; inst++)
    {
        xinputh_interface_t *hid = get_instance(dev_addr, inst);

        if ((ep_addr == hid->ep_in) || (ep_addr == hid->ep_out) || (ep_addr == hid->chatpad_ep_in))
            return inst;
    }

    return 0xff;
}

// Start reading from wired chatpad endpoint
static bool tuh_xinput_receive_chatpad(uint8_t dev_addr, uint8_t instance)
{
    xinputh_interface_t *xid_itf = get_instance(dev_addr, instance);
    if (!xid_itf || !xid_itf->chatpad_ep_in) {
        printf("[CHATPAD] receive_chatpad: no endpoint\n");
        return false;
    }

    if (!usbh_edpt_claim(dev_addr, xid_itf->chatpad_ep_in)) {
        printf("[CHATPAD] receive_chatpad: claim failed for ep 0x%02X\n", xid_itf->chatpad_ep_in);
        return false;
    }

    if (!usbh_edpt_xfer(dev_addr, xid_itf->chatpad_ep_in, xid_itf->chatpad_buf, sizeof(xid_itf->chatpad_buf)))
    {
        printf("[CHATPAD] receive_chatpad: xfer failed\n");
        usbh_edpt_release(dev_addr, xid_itf->chatpad_ep_in);
        return false;
    }
    printf("[CHATPAD] receive_chatpad: started on ep 0x%02X\n", xid_itf->chatpad_ep_in);
    return true;
}

static uint8_t get_instance_id_by_itfnum(uint8_t dev_addr, uint8_t itf)
{
    for (uint8_t inst = 0; inst < CFG_TUH_XINPUT; inst++)
    {
        xinputh_interface_t *hid = get_instance(dev_addr, inst);

        if ((hid->itf_num == itf) && (hid->ep_in || hid->ep_out))
            return inst;
    }

    return 0xff;
}

static void wait_for_tx_complete(uint8_t dev_addr, uint8_t ep_out)
{
    while (usbh_edpt_busy(dev_addr, ep_out))
        tuh_task();
}

// Helper to send control transfer (blocking)
static bool send_ctrl_xfer_sync(uint8_t dev_addr, const tusb_control_request_t* request, uint8_t* buffer)
{
    tuh_xfer_t xfer = {
        .daddr = dev_addr,
        .ep_addr = 0,
        .setup = request,
        .buffer = buffer,
        .complete_cb = NULL,
        .user_data = 0
    };
    return tuh_control_xfer(&xfer);
}

// Initialize wired Xbox 360 chatpad with full sequence
static bool xbox360_wired_chatpad_init(uint8_t dev_addr)
{
    uint8_t buffer[2] = {0};

    printf("[CHATPAD] Wired init sequence starting...\n");

    // Send init sequence
    if (!send_ctrl_xfer_sync(dev_addr, &xbox360_chatpad_init1, NULL)) {
        printf("[CHATPAD] init1 failed\n");
        return false;
    }
    printf("[CHATPAD] init1 OK\n");

    if (!send_ctrl_xfer_sync(dev_addr, &xbox360_chatpad_init2, NULL)) {
        printf("[CHATPAD] init2 failed\n");
        return false;
    }
    printf("[CHATPAD] init2 OK\n");

    if (!send_ctrl_xfer_sync(dev_addr, &xbox360_chatpad_init3, NULL)) {
        printf("[CHATPAD] init3 failed\n");
        return false;
    }
    printf("[CHATPAD] init3 OK\n");

    // Read 2 bytes
    if (!send_ctrl_xfer_sync(dev_addr, &xbox360_chatpad_init4, buffer)) {
        printf("[CHATPAD] init4 (read) failed\n");
        return false;
    }
    printf("[CHATPAD] init4 OK, read: %02X %02X\n", buffer[0], buffer[1]);

    // Write those 2 bytes back
    if (!send_ctrl_xfer_sync(dev_addr, &xbox360_chatpad_init5, buffer)) {
        printf("[CHATPAD] init5 (write) failed\n");
        return false;
    }
    printf("[CHATPAD] init5 OK\n");

    // Read again
    if (!send_ctrl_xfer_sync(dev_addr, &xbox360_chatpad_init6, buffer)) {
        printf("[CHATPAD] init6 (read) failed\n");
        return false;
    }
    printf("[CHATPAD] init6 OK, read: %02X %02X\n", buffer[0], buffer[1]);

    // Enable LEDs on keypress
    if (!send_ctrl_xfer_sync(dev_addr, &xbox360_chatpad_leds_on, NULL)) {
        printf("[CHATPAD] leds_on failed\n");
        return false;
    }
    printf("[CHATPAD] Wired init sequence complete!\n");

    return true;
}

// Send keepalive for wired Xbox 360 chatpad
bool tuh_xinput_wired_chatpad_keepalive(uint8_t dev_addr, uint8_t instance)
{
    xinputh_interface_t *xid_itf = get_instance(dev_addr, instance);
    if (!xid_itf || !xid_itf->chatpad_enabled || !xid_itf->chatpad_inited) {
        return false;
    }

    // Only for wired Xbox 360
    if (xid_itf->type != XBOX360_WIRED) {
        return false;
    }

    const tusb_control_request_t* cmd;
    if (xid_itf->chatpad_stage == 1) {
        cmd = &xbox360_chatpad_keepalive1;
        xid_itf->chatpad_stage = 2;
    } else {
        cmd = &xbox360_chatpad_keepalive2;
        xid_itf->chatpad_stage = 1;
    }

    return send_ctrl_xfer_sync(dev_addr, cmd, NULL);
}

static void xboxone_init( xinputh_interface_t *xid_itf, uint8_t dev_addr, uint8_t instance)
{
    uint16_t PID, VID;
    tuh_vid_pid_get(dev_addr, &VID, &PID);

    tuh_xinput_send_report(dev_addr, instance, xboxone_power_on, sizeof(xboxone_power_on));
    wait_for_tx_complete(dev_addr, xid_itf->ep_out);
    tuh_xinput_send_report(dev_addr, instance, xboxone_s_init, sizeof(xboxone_s_init));
    wait_for_tx_complete(dev_addr, xid_itf->ep_out);

    if (VID == 0x045e && (PID == 0x0b00))
    {
        tuh_xinput_send_report(dev_addr, instance, extra_input_packet_init, sizeof(extra_input_packet_init));
        wait_for_tx_complete(dev_addr, xid_itf->ep_out);
    }

    //Required for PDP aftermarket controllers
    if (VID == 0x0e6f)
    {
        tuh_xinput_send_report(dev_addr, instance, xboxone_pdp_led_on, sizeof(xboxone_pdp_led_on));
        wait_for_tx_complete(dev_addr, xid_itf->ep_out);
        tuh_xinput_send_report(dev_addr, instance, xboxone_pdp_auth, sizeof(xboxone_pdp_auth));
        wait_for_tx_complete(dev_addr, xid_itf->ep_out);
    }
}

bool tuh_xinput_receive_report(uint8_t dev_addr, uint8_t instance)
{
    xinputh_interface_t *xid_itf = get_instance(dev_addr, instance);
    TU_VERIFY(usbh_edpt_claim(dev_addr, xid_itf->ep_in));

    if ( !usbh_edpt_xfer(dev_addr, xid_itf->ep_in, xid_itf->epin_buf, xid_itf->epin_size) )
    {
        usbh_edpt_release(dev_addr, xid_itf->ep_in);
        return false;
    }
    return true;
}

bool tuh_xinput_send_report(uint8_t dev_addr, uint8_t instance, const uint8_t *txbuf, uint16_t len)
{
    xinputh_interface_t *xid_itf = get_instance(dev_addr, instance);

    TU_ASSERT(len <= xid_itf->epout_size);
    TU_VERIFY(usbh_edpt_claim(dev_addr, xid_itf->ep_out));

    memcpy(xid_itf->epout_buf, txbuf, len);

    if ( !usbh_edpt_xfer(dev_addr, xid_itf->ep_out, xid_itf->epout_buf, len))
    {
        usbh_edpt_release(dev_addr, xid_itf->ep_out);
        return false;
    }
    return true;
}

bool tuh_xinput_set_led(uint8_t dev_addr, uint8_t instance, uint8_t quadrant, bool block)
{
    xinputh_interface_t *xid_itf = get_instance(dev_addr, instance);
    uint8_t txbuf[32];
    uint16_t len;
    switch (xid_itf->type)
    {
    case XBOX360_WIRELESS:
        memcpy(txbuf, xbox360w_led, sizeof(xbox360w_led));
        txbuf[3] = (quadrant == 0) ? 0x40 : (0x40 | (quadrant + 5));
        len = sizeof(xbox360w_led);
        break;
    case XBOX360_WIRED:
        memcpy(txbuf, xbox360_wired_led, sizeof(xbox360_wired_led));
        txbuf[2] = (quadrant == 0) ? 0 : (quadrant + 5);
        len = sizeof(xbox360_wired_led);
        break;
    default:
        return true;
    }
    bool ret = tuh_xinput_send_report(dev_addr, instance, txbuf, len);
    if (block && ret)
    {
        wait_for_tx_complete(dev_addr, xid_itf->ep_out);
    }
    return ret;
}

bool tuh_xinput_set_rumble(uint8_t dev_addr, uint8_t instance, uint8_t lValue, uint8_t rValue, bool block)
{
    xinputh_interface_t *xid_itf = get_instance(dev_addr, instance);
    uint8_t txbuf[32];
    uint16_t len;

    switch (xid_itf->type)
    {
    case XBOX360_WIRELESS:
        memcpy(txbuf, xbox360w_rumble, sizeof(xbox360w_rumble));
        txbuf[5] = lValue;
        txbuf[6] = rValue;
        len = sizeof(xbox360w_rumble);
        break;
    case XBOX360_WIRED:
        memcpy(txbuf, xbox360_wired_rumble, sizeof(xbox360_wired_rumble));
        txbuf[3] = lValue;
        txbuf[4] = rValue;
        len = sizeof(xbox360_wired_rumble);
        break;
    case XBOXONE:
        memcpy(txbuf, xboxone_rumble, sizeof(xboxone_rumble));
        txbuf[8] = lValue / 2; // 0 - 128
        txbuf[9] = rValue / 2; // 0 - 128
        len = sizeof(xboxone_rumble);
        break;
    case XBOXOG:
        memcpy(txbuf, xboxog_rumble, sizeof(xboxog_rumble));
        txbuf[2] = lValue;
        txbuf[3] = lValue;
        txbuf[4] = rValue;
        txbuf[5] = rValue;
        len = sizeof(xboxog_rumble);
        break;
    default:
        return true;
    }
    bool ret = tuh_xinput_send_report(dev_addr, instance, txbuf, len);
    if (block && ret)
    {
        wait_for_tx_complete(dev_addr, xid_itf->ep_out);
    }
    return true;
}

bool tuh_xinput_init_chatpad(uint8_t dev_addr, uint8_t instance, bool enable)
{
    xinputh_interface_t *xid_itf = get_instance(dev_addr, instance);

    // Chatpad only supported on Xbox 360 Wireless
    if (xid_itf->type != XBOX360_WIRELESS)
    {
        return false;
    }

    xid_itf->chatpad_enabled = enable;

    if (!enable)
    {
        xid_itf->chatpad_inited = false;
        return true;
    }

    printf("[CHATPAD] Sending chatpad init command\n");

    // Send chatpad init command
    bool ret = tuh_xinput_send_report(dev_addr, instance, xbox360w_chatpad_init, sizeof(xbox360w_chatpad_init));
    if (ret)
    {
        wait_for_tx_complete(dev_addr, xid_itf->ep_out);
        xid_itf->chatpad_inited = true;
        xid_itf->chatpad_stage = 1;  // Start with keepalive1
        xid_itf->chatpad_last_time = 0;
    }
    return ret;
}

bool tuh_xinput_chatpad_keepalive(uint8_t dev_addr, uint8_t instance)
{
    xinputh_interface_t *xid_itf = get_instance(dev_addr, instance);
    if (!xid_itf) return false;

    if (!xid_itf->chatpad_enabled || !xid_itf->chatpad_inited)
    {
        return false;
    }

    // Wired Xbox 360 uses control transfers for keepalive
    if (xid_itf->type == XBOX360_WIRED)
    {
        const tusb_control_request_t* cmd;
        if (xid_itf->chatpad_stage == 1) {
            cmd = &xbox360_chatpad_keepalive1;
            xid_itf->chatpad_stage = 2;
        } else {
            cmd = &xbox360_chatpad_keepalive2;
            xid_itf->chatpad_stage = 1;
        }
        static uint32_t keepalive_count = 0;
        ++keepalive_count;
        bool ok = send_ctrl_xfer_sync(dev_addr, cmd, NULL);

        // Every 5 keepalives, check endpoint status and try polling via control transfer
        if (keepalive_count % 5 == 0) {
            bool ep_busy = usbh_edpt_busy(dev_addr, xid_itf->chatpad_ep_in);
            printf("[CHATPAD] keepalive #%lu ep_busy=%d\n", (unsigned long)keepalive_count, ep_busy);

            // Try reading chatpad status via control transfer (GET_STATUS on interface 2)
            static uint8_t status_buf[8] = {0};
            tusb_control_request_t get_status = {
                .bmRequestType = 0xC1,  // Vendor, Device-to-Host, Interface
                .bRequest = 0x01,
                .wValue = 0x0100,
                .wIndex = 0x0002,  // Interface 2
                .wLength = 8
            };
            if (send_ctrl_xfer_sync(dev_addr, &get_status, status_buf)) {
                bool has_data = false;
                for (int i = 0; i < 8; i++) if (status_buf[i]) has_data = true;
                if (has_data) {
                    printf("[CHATPAD] CTRL poll:");
                    for (int i = 0; i < 8; i++) printf(" %02X", status_buf[i]);
                    printf("\n");
                }
            }

            // If endpoint not busy, try re-queue the receive
            if (!ep_busy) {
                printf("[CHATPAD] Re-queuing chatpad receive...\n");
                tuh_xinput_receive_chatpad(dev_addr, instance);
            }
        }
        return ok;
    }

    // Wireless Xbox 360 uses endpoint reports for keepalive
    const uint8_t *cmd;
    uint16_t len;

    if (xid_itf->chatpad_stage == 1)
    {
        cmd = xbox360w_chatpad_keepalive1;
        len = sizeof(xbox360w_chatpad_keepalive1);
        xid_itf->chatpad_stage = 2;
    }
    else
    {
        cmd = xbox360w_chatpad_keepalive2;
        len = sizeof(xbox360w_chatpad_keepalive2);
        xid_itf->chatpad_stage = 1;
    }

    return tuh_xinput_send_report(dev_addr, instance, cmd, len);
}

//--------------------------------------------------------------------+
// USBH API
//--------------------------------------------------------------------+
bool xinputh_init(void)
{
    tu_memclr(_xinputh_dev, sizeof(_xinputh_dev));
    return true;
}

bool xinputh_open(uint8_t rhport, uint8_t dev_addr, tusb_desc_interface_t const *desc_itf, uint16_t max_len)
{
    TU_VERIFY(dev_addr <= CFG_TUH_DEVICE_MAX);

    // Log device version for chatpad endpoint selection
    uint16_t vid, pid;
    tuh_vid_pid_get(dev_addr, &vid, &pid);

    // Debug: log all interfaces offered
    printf("[XINPUT] Interface %d: Class=0x%02X SubClass=0x%02X Protocol=0x%02X Endpoints=%d (VID=%04X PID=%04X)\n",
           desc_itf->bInterfaceNumber, desc_itf->bInterfaceClass,
           desc_itf->bInterfaceSubClass, desc_itf->bInterfaceProtocol,
           desc_itf->bNumEndpoints, vid, pid);

    xinput_type_t type = XINPUT_UNKNOWN;

    // Log all endpoints for interfaces we don't fully handle (like headset interface 1)
    // Also try opening interface 1's IN endpoints to see if chatpad data comes through there
    if (desc_itf->bInterfaceNumber == 1 && desc_itf->bNumEndpoints > 0) {
        printf("[XINPUT] Interface 1 (headset/expansion) endpoints:\n");
        uint8_t const *p_desc = (uint8_t const *)desc_itf;
        int pos = 0;
        while (pos < max_len) {
            if (tu_desc_type(p_desc) == TUSB_DESC_ENDPOINT) {
                tusb_desc_endpoint_t const *desc_ep = (tusb_desc_endpoint_t const *)p_desc;
                printf("  EP 0x%02X, MaxPkt=%d, Interval=%d\n",
                       desc_ep->bEndpointAddress,
                       tu_edpt_packet_size(desc_ep),
                       desc_ep->bInterval);
                // Try opening IN endpoints and queue receives - chatpad might use expansion port
                if (tu_edpt_dir(desc_ep->bEndpointAddress) == TUSB_DIR_IN) {
                    if (tuh_edpt_open(dev_addr, desc_ep)) {
                        printf("  -> Opened EP 0x%02X for chatpad test\n", desc_ep->bEndpointAddress);
                        // Queue a receive to see if data comes through
                        static uint8_t expansion_buf[32];
                        if (usbh_edpt_xfer(dev_addr, desc_ep->bEndpointAddress, expansion_buf, 32)) {
                            printf("  -> Queued receive on EP 0x%02X\n", desc_ep->bEndpointAddress);
                        }
                    }
                }
            }
            pos += tu_desc_len(p_desc);
            p_desc = tu_desc_next(p_desc);
        }
    }

    // Check for wired chatpad interface (Interface 2, SubClass 0x5D, Protocol 0x02)
    if (desc_itf->bInterfaceSubClass == 0x5D && desc_itf->bInterfaceProtocol == 0x02 &&
        desc_itf->bNumEndpoints == 1)
    {
        // This is the wired chatpad interface - associate with existing gamepad instance
        xinputh_device_t *xinput_dev = get_dev(dev_addr);
        if (xinput_dev->inst_count > 0)
        {
            // Find the wired gamepad instance and add chatpad endpoint
            for (uint8_t i = 0; i < xinput_dev->inst_count; i++)
            {
                xinputh_interface_t *xid_itf = get_instance(dev_addr, i);
                if (xid_itf->type == XBOX360_WIRED)
                {
                    // Parse the chatpad endpoint
                    uint8_t const *p_desc = (uint8_t const *)desc_itf;
                    int pos = 0;
                    while (pos < max_len)
                    {
                        if (tu_desc_type(p_desc) == TUSB_DESC_ENDPOINT)
                        {
                            tusb_desc_endpoint_t const *desc_ep = (tusb_desc_endpoint_t const *)p_desc;
                            printf("[CHATPAD] Interface 2 EP 0x%02X, MaxPkt=%d, Interval=%d\n",
                                   desc_ep->bEndpointAddress,
                                   tu_edpt_packet_size(desc_ep),
                                   desc_ep->bInterval);
                            if (tu_edpt_dir(desc_ep->bEndpointAddress) == TUSB_DIR_IN)
                            {
                                TU_ASSERT(tuh_edpt_open(dev_addr, desc_ep));
                                xid_itf->chatpad_ep_in = desc_ep->bEndpointAddress;
                                xid_itf->chatpad_itf_num = desc_itf->bInterfaceNumber;
                                xid_itf->chatpad_enabled = true;
                                printf("[CHATPAD] Wired chatpad endpoint 0x%02X opened, init deferred to set_config\n", desc_ep->bEndpointAddress);
                                // Full init sequence will be done in set_config
                            }
                            break;
                        }
                        pos += tu_desc_len(p_desc);
                        p_desc = tu_desc_next(p_desc);
                    }
                    return true;  // Claim the interface but don't create new instance
                }
            }
        }
        return false;  // No wired gamepad found to attach to
    }

    if (desc_itf->bNumEndpoints < 2)
        type = XINPUT_UNKNOWN;
    else if (desc_itf->bInterfaceSubClass == 0x5D && //Xbox360 wireless bInterfaceSubClass
             desc_itf->bInterfaceProtocol == 0x81)   //Xbox360 wireless bInterfaceProtocol
        type = XBOX360_WIRELESS;
    else if (desc_itf->bInterfaceSubClass == 0x5D && //Xbox360 wired bInterfaceSubClass
             desc_itf->bInterfaceProtocol == 0x01)   //Xbox360 wired bInterfaceProtocol
        type = XBOX360_WIRED;
    else if (desc_itf->bInterfaceSubClass == 0x47 && //Xbone and SX bInterfaceSubClass
             desc_itf->bInterfaceProtocol == 0xD0)   //Xbone and SX bInterfaceProtocol
        type = XBOXONE;
    else if (desc_itf->bInterfaceClass == 0x58 &&  //XboxOG bInterfaceClass
             desc_itf->bInterfaceSubClass == 0x42) //XboxOG bInterfaceSubClass
        type = XBOXOG;

    if (type == XINPUT_UNKNOWN)
    {
        TU_LOG2("XINPUT: Not a valid interface\n");
        return false;
    }

    printf("[XINPUT] Opening type=%d dev_addr=%d itf=%d\n", type, dev_addr, desc_itf->bInterfaceNumber);

    xinputh_device_t *xinput_dev = get_dev(dev_addr);
    if (!xinput_dev) {
        printf("[XINPUT] ERROR: get_dev returned NULL\n");
        return false;
    }

    printf("[XINPUT] inst_count=%d max=%d\n", xinput_dev->inst_count, CFG_TUH_XINPUT);
    TU_ASSERT(xinput_dev->inst_count < CFG_TUH_XINPUT, 0);

    xinputh_interface_t *xid_itf = get_instance(dev_addr, xinput_dev->inst_count);
    if (!xid_itf) {
        printf("[XINPUT] ERROR: get_instance returned NULL\n");
        return false;
    }
    xid_itf->itf_num = desc_itf->bInterfaceNumber;
    xid_itf->type = type;

    //Parse descriptor for all endpoints and open them
    uint8_t const *p_desc = (uint8_t const *)desc_itf;
    int endpoint = 0;
    int pos = 0;
    while (endpoint < desc_itf->bNumEndpoints && pos < max_len)
    {
        if (tu_desc_type(p_desc) != TUSB_DESC_ENDPOINT)
        {
            pos += tu_desc_len(p_desc);
            p_desc = tu_desc_next(p_desc);
            continue;
        }
        tusb_desc_endpoint_t const *desc_ep = (tusb_desc_endpoint_t const *)p_desc;
        TU_ASSERT(TUSB_DESC_ENDPOINT == desc_ep->bDescriptorType);
        TU_ASSERT(tuh_edpt_open(dev_addr, desc_ep));
        if (tu_edpt_dir(desc_ep->bEndpointAddress) == TUSB_DIR_OUT)
        {
            xid_itf->ep_out = desc_ep->bEndpointAddress;
            xid_itf->epout_size = tu_edpt_packet_size(desc_ep);
        }
        else
        {
            xid_itf->ep_in = desc_ep->bEndpointAddress;
            xid_itf->epin_size = tu_edpt_packet_size(desc_ep);
        }
        endpoint++;
        pos += tu_desc_len(p_desc);
        p_desc = tu_desc_next(p_desc);
    }

    xinput_dev->inst_count++;
    return true;
}

bool xinputh_set_config(uint8_t dev_addr, uint8_t itf_num)
{
    printf("[XINPUT] set_config dev_addr=%d itf_num=%d\n", dev_addr, itf_num);
    uint8_t instance = get_instance_id_by_itfnum(dev_addr, itf_num);

    // Check if this is a chatpad interface (attached to an existing gamepad instance)
    if (instance == 0xff) {
        // Look for a gamepad instance that has this as its chatpad interface
        xinputh_device_t *xinput_dev = get_dev(dev_addr);
        if (xinput_dev) {
            for (uint8_t i = 0; i < xinput_dev->inst_count; i++) {
                xinputh_interface_t *itf = get_instance(dev_addr, i);
                if (itf && itf->chatpad_itf_num == itf_num) {
                    printf("[XINPUT] set_config: chatpad interface %d attached to instance %d\n", itf_num, i);
                    // Run full wired chatpad initialization sequence
                    if (itf->chatpad_enabled && !itf->chatpad_inited) {
                        if (xbox360_wired_chatpad_init(dev_addr)) {
                            itf->chatpad_inited = true;
                            itf->chatpad_stage = 1;
                            // Start reading from chatpad endpoint
                            tuh_xinput_receive_chatpad(dev_addr, i);
                        }
                    }
                    usbh_driver_set_config_complete(dev_addr, itf_num);
                    return true;
                }
            }
        }
        printf("[XINPUT] set_config: no instance found for itf_num=%d (not a chatpad either)\n", itf_num);
        return false;
    }

    xinputh_interface_t *xid_itf = get_instance(dev_addr, instance);
    if (!xid_itf) {
        printf("[XINPUT] ERROR: get_instance returned NULL in set_config\n");
        return false;
    }
    xid_itf->connected = true;

    if (xid_itf->type == XBOX360_WIRELESS)
    {
        //Wireless controllers may not be connected yet.
        xid_itf->connected = false;
        tuh_xinput_send_report(dev_addr, instance, xbox360w_inquire_present, sizeof(xbox360w_inquire_present));
        wait_for_tx_complete(dev_addr, xid_itf->ep_out);
    }
    else if (xid_itf->type == XBOX360_WIRED)
    {
        // Wired chatpad init will be done when chatpad interface is configured
    }
    else if (xid_itf->type == XBOXONE)
    {
        xboxone_init(xid_itf, dev_addr, instance);
    }

    if (tuh_xinput_mount_cb)
    {
        tuh_xinput_mount_cb(dev_addr, instance, xid_itf);
    }

    usbh_driver_set_config_complete(dev_addr, xid_itf->itf_num);
    return true;
}

bool xinputh_xfer_cb(uint8_t dev_addr, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes)
{
    // Debug: log ALL IN transfers on any endpoint to find chatpad data
    if (tu_edpt_dir(ep_addr) == TUSB_DIR_IN && ep_addr != 0x81) {
        printf("[XFER] ep=0x%02X result=%d bytes=%lu\n",
               ep_addr, result, (unsigned long)xferred_bytes);
    }

    uint8_t const dir = tu_edpt_dir(ep_addr);
    uint8_t const instance = get_instance_id_by_epaddr(dev_addr, ep_addr);
    if (instance == 0xff) {
        printf("[XINPUT] xfer_cb: no instance for ep_addr=0x%02X\n", ep_addr);
        return false;
    }
    xinputh_interface_t *xid_itf = get_instance(dev_addr, instance);
    if (!xid_itf) {
        printf("[XINPUT] xfer_cb: get_instance returned NULL\n");
        return false;
    }
    xinput_gamepad_t *pad = &xid_itf->pad;
    uint8_t *rdata = xid_itf->epin_buf;

    xid_itf->last_xfer_result = result;
    xid_itf->last_xferred_bytes = xferred_bytes;

    // On transfer error, bail early but notify the application
    if (result != XFER_RESULT_SUCCESS)
    {
        if (dir == TUSB_DIR_IN)
        {
            tuh_xinput_report_received_cb(dev_addr, instance, xid_itf, sizeof(xinputh_interface_t));
        }
        else if (tuh_xinput_report_sent_cb)
        {
            tuh_xinput_report_sent_cb(dev_addr, instance, xid_itf->epout_buf, xferred_bytes);
        }
        return false;
    }

    if (dir == TUSB_DIR_IN)
    {
        TU_LOG2("Get Report callback (%u, %u, %u bytes)\r\n", dev_addr, instance, xferred_bytes);

        // Check if this is from wired chatpad endpoint
        if (ep_addr == xid_itf->chatpad_ep_in)
        {
            // Wired chatpad data - dump all bytes
            uint8_t *cdata = xid_itf->chatpad_buf;
            printf("[CHATPAD] Wired data (%lu bytes): ", (unsigned long)xferred_bytes);
            for (uint32_t i = 0; i < xferred_bytes && i < 8; i++) {
                printf("%02X ", cdata[i]);
            }
            printf("\n");

            // Check for actual key data (not status messages)
            // Status messages start with F0, key data typically starts with 00
            if (cdata[0] == 0x00) {
                // Key data format: 00 MODIFIER KEY1 KEY2 00
                if (cdata[1] != xid_itf->chatpad_data[0] ||
                    cdata[2] != xid_itf->chatpad_data[1] ||
                    cdata[3] != xid_itf->chatpad_data[2])
                {
                    xid_itf->chatpad_data[0] = cdata[1];  // modifier
                    xid_itf->chatpad_data[1] = cdata[2];  // key1
                    xid_itf->chatpad_data[2] = cdata[3];  // key2
                    xid_itf->new_chatpad_data = true;
                    printf("[CHATPAD] KEY: mod=%02X key1=%02X key2=%02X\n",
                           cdata[1], cdata[2], cdata[3]);
                }
            } else if (cdata[0] == 0xF0 && cdata[1] == 0x03) {
                // F0 03 = chatpad connected/ready - must send 0x1B to enable key reporting
                printf("[CHATPAD] Status F0 03: chatpad ready, sending LED enable\n");
                send_ctrl_xfer_sync(dev_addr, &xbox360_chatpad_leds_on, NULL);
            } else {
                printf("[CHATPAD] Unknown message (0x%02X 0x%02X)\n", cdata[0], cdata[1]);
            }
            // Re-queue chatpad read immediately
            if (!tuh_xinput_receive_chatpad(dev_addr, instance)) {
                printf("[CHATPAD] WARNING: re-queue failed!\n");
            }
            return true;
        }

        if (xid_itf->type == XBOX360_WIRED)
        {
            #define GET_USHORT(a) (uint16_t)((a)[1] << 8 | (a)[0])
            #define GET_SHORT(a) ((int16_t)GET_USHORT(a))

            // Chatpad key data - look for various possible formats:
            // - Serial protocol uses 0xB4 for key messages
            // - USB might use type 0x02, 0xB4, or other markers
            // Check for chatpad-like reports (anything not gamepad 0x14 or LED 0x03)
            if (rdata[1] != 0x14 && rdata[1] != 0x03) {
                printf("[360WIRED] CHATPAD? type=0x%02X len=%lu:", rdata[1], (unsigned long)xferred_bytes);
                for (uint32_t i = 0; i < xferred_bytes && i < 32; i++) printf(" %02X", rdata[i]);
                printf("\n");
            }

            // Check for chatpad data in multiple possible formats
            // Format 1: Report type 0x02 (like wireless chatpad in gamepad report)
            // Format 2: Report starting with 0x00 (chatpad EP format)
            // Format 3: Report type matching serial 0xB4
            bool is_chatpad = false;
            uint8_t modifier = 0, key1 = 0, key2 = 0;

            if (rdata[0] == 0x00 && xferred_bytes >= 5 && rdata[1] != 0x14 && rdata[1] != 0x03) {
                // Format: 00 XX modifier key1 key2
                modifier = rdata[2];
                key1 = rdata[3];
                key2 = rdata[4];
                is_chatpad = true;
            } else if (rdata[0] == 0x02 && xferred_bytes >= 5) {
                // Format: 02 XX modifier key1 key2
                modifier = rdata[2];
                key1 = rdata[3];
                key2 = rdata[4];
                is_chatpad = true;
            }

            if (is_chatpad && (modifier != 0 || key1 != 0 || key2 != 0)) {
                if (modifier != xid_itf->chatpad_data[0] ||
                    key1 != xid_itf->chatpad_data[1] ||
                    key2 != xid_itf->chatpad_data[2])
                {
                    xid_itf->chatpad_data[0] = modifier;
                    xid_itf->chatpad_data[1] = key1;
                    xid_itf->chatpad_data[2] = key2;
                    xid_itf->new_chatpad_data = true;
                    printf("[CHATPAD] KEY: mod=%02X key1=%02X key2=%02X\n",
                           modifier, key1, key2);
                }
            }

            if (rdata[1] == 0x14)
            {
                // Log if report is longer than standard 20 bytes - chatpad data may be appended!
                if (xferred_bytes > 20) {
                    printf("[360WIRED] Extended report len=%lu:", (unsigned long)xferred_bytes);
                    for (uint32_t i = 20; i < xferred_bytes && i < 40; i++) printf(" %02X", rdata[i]);
                    printf("\n");
                }
                // Check for chatpad data at end of extended report (like wireless)
                if (xferred_bytes >= 27 && xid_itf->chatpad_enabled) {
                    uint8_t chatpad_status = rdata[24];
                    if (chatpad_status == 0x00) {
                        // Key data
                        uint8_t mod = rdata[25];
                        uint8_t key1 = rdata[26];
                        uint8_t key2 = (xferred_bytes > 27) ? rdata[27] : 0;
                        if (mod != xid_itf->chatpad_data[0] || key1 != xid_itf->chatpad_data[1] || key2 != xid_itf->chatpad_data[2]) {
                            xid_itf->chatpad_data[0] = mod;
                            xid_itf->chatpad_data[1] = key1;
                            xid_itf->chatpad_data[2] = key2;
                            xid_itf->new_chatpad_data = true;
                            printf("[CHATPAD] WIRED KEY: mod=%02X key1=%02X key2=%02X\n", mod, key1, key2);
                        }
                    }
                }

                tu_memclr(pad, sizeof(xinput_gamepad_t));
                uint16_t wButtons = rdata[3] << 8 | rdata[2];

                //Map digital buttons
                if (wButtons & (1 << 0)) pad->wButtons |= XINPUT_GAMEPAD_DPAD_UP;
                if (wButtons & (1 << 1)) pad->wButtons |= XINPUT_GAMEPAD_DPAD_DOWN;
                if (wButtons & (1 << 2)) pad->wButtons |= XINPUT_GAMEPAD_DPAD_LEFT;
                if (wButtons & (1 << 3)) pad->wButtons |= XINPUT_GAMEPAD_DPAD_RIGHT;
                if (wButtons & (1 << 4)) pad->wButtons |= XINPUT_GAMEPAD_START;
                if (wButtons & (1 << 5)) pad->wButtons |= XINPUT_GAMEPAD_BACK;
                if (wButtons & (1 << 6)) pad->wButtons |= XINPUT_GAMEPAD_LEFT_THUMB;
                if (wButtons & (1 << 7)) pad->wButtons |= XINPUT_GAMEPAD_RIGHT_THUMB;
                if (wButtons & (1 << 8)) pad->wButtons |= XINPUT_GAMEPAD_LEFT_SHOULDER;
                if (wButtons & (1 << 9)) pad->wButtons |= XINPUT_GAMEPAD_RIGHT_SHOULDER;
                if (wButtons & (1 << 10)) pad->wButtons |= XINPUT_GAMEPAD_GUIDE;
                if (wButtons & (1 << 12)) pad->wButtons |= XINPUT_GAMEPAD_A;
                if (wButtons & (1 << 13)) pad->wButtons |= XINPUT_GAMEPAD_B;
                if (wButtons & (1 << 14)) pad->wButtons |= XINPUT_GAMEPAD_X;
                if (wButtons & (1 << 15)) pad->wButtons |= XINPUT_GAMEPAD_Y;

                //Map the left and right triggers
                pad->bLeftTrigger = rdata[4];
                pad->bRightTrigger = rdata[5];

                //Map analog sticks
                pad->sThumbLX = rdata[7] << 8 | rdata[6];
                pad->sThumbLY = rdata[9] << 8 | rdata[8];
                pad->sThumbRX = rdata[11] << 8 | rdata[10];
                pad->sThumbRY = rdata[13] << 8 | rdata[12];

                xid_itf->new_pad_data = true;
            }           
        }
        else if (xid_itf->type == XBOX360_WIRELESS)
        {
            //Connect/Disconnect packet
            if (rdata[0] & 0x08)
            {
                if (rdata[1] != 0x00 && xid_itf->connected == false)
                {
                    TU_LOG2("XINPUT: WIRELESS CONTROLLER CONNECTED\n");
                    xid_itf->connected = true;
                }
                else if (rdata[1] == 0x00 && xid_itf->connected == true)
                {
                    TU_LOG2("XINPUT: WIRELESS CONTROLLER DISCONNECTED\n");
                    xid_itf->connected = false;
                    xid_itf->chatpad_inited = false;
                }
            }

            //Button status packet
            if ((rdata[1] & 1) && rdata[5] == 0x13)
            {
                tu_memclr(pad, sizeof(xinput_gamepad_t));
                uint16_t wButtons = rdata[7] << 8 | rdata[6];

                //Map digital buttons
                if (wButtons & (1 << 0)) pad->wButtons |= XINPUT_GAMEPAD_DPAD_UP;
                if (wButtons & (1 << 1)) pad->wButtons |= XINPUT_GAMEPAD_DPAD_DOWN;
                if (wButtons & (1 << 2)) pad->wButtons |= XINPUT_GAMEPAD_DPAD_LEFT;
                if (wButtons & (1 << 3)) pad->wButtons |= XINPUT_GAMEPAD_DPAD_RIGHT;
                if (wButtons & (1 << 4)) pad->wButtons |= XINPUT_GAMEPAD_START;
                if (wButtons & (1 << 5)) pad->wButtons |= XINPUT_GAMEPAD_BACK;
                if (wButtons & (1 << 6)) pad->wButtons |= XINPUT_GAMEPAD_LEFT_THUMB;
                if (wButtons & (1 << 7)) pad->wButtons |= XINPUT_GAMEPAD_RIGHT_THUMB;
                if (wButtons & (1 << 8)) pad->wButtons |= XINPUT_GAMEPAD_LEFT_SHOULDER;
                if (wButtons & (1 << 9)) pad->wButtons |= XINPUT_GAMEPAD_RIGHT_SHOULDER;
                if (wButtons & (1 << 10)) pad->wButtons |= XINPUT_GAMEPAD_GUIDE;
                if (wButtons & (1 << 12)) pad->wButtons |= XINPUT_GAMEPAD_A;
                if (wButtons & (1 << 13)) pad->wButtons |= XINPUT_GAMEPAD_B;
                if (wButtons & (1 << 14)) pad->wButtons |= XINPUT_GAMEPAD_X;
                if (wButtons & (1 << 15)) pad->wButtons |= XINPUT_GAMEPAD_Y;

                //Map the left and right triggers
                pad->bLeftTrigger = rdata[8];
                pad->bRightTrigger = rdata[9];

                //Map analog sticks
                pad->sThumbLX = rdata[11] << 8 | rdata[10];
                pad->sThumbLY = rdata[13] << 8 | rdata[12];
                pad->sThumbRX = rdata[15] << 8 | rdata[14];
                pad->sThumbRY = rdata[17] << 8 | rdata[16];

                xid_itf->new_pad_data = true;

                // Chatpad data (if enabled and report is long enough)
                if (xid_itf->chatpad_enabled && xferred_bytes >= 27)
                {
                    uint8_t chatpad_status = rdata[24];

                    if (chatpad_status == 0x00)
                    {
                        // Chatpad key data: [25]=modifier, [26]=key1, [27]=key2
                        if (rdata[25] != xid_itf->chatpad_data[0] ||
                            rdata[26] != xid_itf->chatpad_data[1] ||
                            rdata[27] != xid_itf->chatpad_data[2])
                        {
                            xid_itf->chatpad_data[0] = rdata[25];  // modifier
                            xid_itf->chatpad_data[1] = rdata[26];  // key1
                            xid_itf->chatpad_data[2] = rdata[27];  // key2
                            xid_itf->new_chatpad_data = true;
                            printf("[CHATPAD] mod=%02X key1=%02X key2=%02X\n",
                                   rdata[25], rdata[26], rdata[27]);
                        }
                    }
                    else if (chatpad_status == 0xF0 && rdata[25] == 0x03)
                    {
                        // Chatpad connected notification - reinit if needed
                        if (!xid_itf->chatpad_inited)
                        {
                            printf("[CHATPAD] Chatpad connected, initializing\n");
                            tuh_xinput_init_chatpad(dev_addr, instance, true);
                        }
                    }
                }
            }
        }
        else if (xid_itf->type == XBOXONE)
        {
            if (rdata[0] == GIP_CMD_INPUT)
            {
                tu_memclr(pad, sizeof(xinput_gamepad_t));
                uint16_t wButtons = rdata[5] << 8 | rdata[4];

                //Map digital buttons
                if (wButtons & (1 << 8)) pad->wButtons |= XINPUT_GAMEPAD_DPAD_UP;
                if (wButtons & (1 << 9)) pad->wButtons |= XINPUT_GAMEPAD_DPAD_DOWN;
                if (wButtons & (1 << 10)) pad->wButtons |= XINPUT_GAMEPAD_DPAD_LEFT;
                if (wButtons & (1 << 11)) pad->wButtons |= XINPUT_GAMEPAD_DPAD_RIGHT;
                if (wButtons & (1 << 2)) pad->wButtons |= XINPUT_GAMEPAD_START;
                if (wButtons & (1 << 3)) pad->wButtons |= XINPUT_GAMEPAD_BACK;
                if (wButtons & (1 << 14)) pad->wButtons |= XINPUT_GAMEPAD_LEFT_THUMB;
                if (wButtons & (1 << 15)) pad->wButtons |= XINPUT_GAMEPAD_RIGHT_THUMB;
                if (wButtons & (1 << 12)) pad->wButtons |= XINPUT_GAMEPAD_LEFT_SHOULDER;
                if (wButtons & (1 << 13)) pad->wButtons |= XINPUT_GAMEPAD_RIGHT_SHOULDER;
                if (wButtons & (1 << 4)) pad->wButtons |= XINPUT_GAMEPAD_A;
                if (wButtons & (1 << 5)) pad->wButtons |= XINPUT_GAMEPAD_B;
                if (wButtons & (1 << 6)) pad->wButtons |= XINPUT_GAMEPAD_X;
                if (wButtons & (1 << 7)) pad->wButtons |= XINPUT_GAMEPAD_Y;
                if (rdata[22] && 0x01) pad->wButtons   |= XINPUT_GAMEPAD_SHARE;

                //Map the left and right triggers
                pad->bLeftTrigger = (rdata[7] << 8 | rdata[6]) >> 2;
                pad->bRightTrigger = (rdata[9] << 8 | rdata[8]) >> 2;

                //Map analog sticks
                pad->sThumbLX = rdata[11] << 8 | rdata[10];
                pad->sThumbLY = rdata[13] << 8 | rdata[12];
                pad->sThumbRX = rdata[15] << 8 | rdata[14];
                pad->sThumbRY = rdata[17] << 8 | rdata[16];

                xid_itf->new_pad_data = true;
            }
            else if (rdata[0] == GIP_CMD_VIRTUAL_KEY)
            {
                if (rdata[4] == 0x01 && !(pad->wButtons & XINPUT_GAMEPAD_GUIDE)) {
                    xid_itf->new_pad_data = true;
                    pad->wButtons |= XINPUT_GAMEPAD_GUIDE;
                }
                else if (rdata[4] == 0x00 && (pad->wButtons & XINPUT_GAMEPAD_GUIDE)) {
                    xid_itf->new_pad_data = true;
                    pad->wButtons &= ~XINPUT_GAMEPAD_GUIDE;
                }
            }
            else if (rdata[0] == GIP_CMD_ANNOUNCE)
            {
                xboxone_init(xid_itf, dev_addr, instance);
            }
        }
        else if (xid_itf->type == XBOXOG)
        {
            if (rdata[1] == 0x14)
            {
                tu_memclr(pad, sizeof(xinput_gamepad_t));
                uint16_t wButtons = rdata[3] << 8 | rdata[2];

                //Map digital buttons
                if (wButtons & (1 << 0)) pad->wButtons |= XINPUT_GAMEPAD_DPAD_UP;
                if (wButtons & (1 << 1)) pad->wButtons |= XINPUT_GAMEPAD_DPAD_DOWN;
                if (wButtons & (1 << 2)) pad->wButtons |= XINPUT_GAMEPAD_DPAD_LEFT;
                if (wButtons & (1 << 3)) pad->wButtons |= XINPUT_GAMEPAD_DPAD_RIGHT;
                if (wButtons & (1 << 4)) pad->wButtons |= XINPUT_GAMEPAD_START;
                if (wButtons & (1 << 5)) pad->wButtons |= XINPUT_GAMEPAD_BACK;
                if (wButtons & (1 << 6)) pad->wButtons |= XINPUT_GAMEPAD_LEFT_THUMB;
                if (wButtons & (1 << 7)) pad->wButtons |= XINPUT_GAMEPAD_RIGHT_THUMB;

                if (rdata[4] > 0x20) pad->wButtons |= XINPUT_GAMEPAD_A;
                if (rdata[5] > 0x20) pad->wButtons |= XINPUT_GAMEPAD_B;
                if (rdata[6] > 0x20) pad->wButtons |= XINPUT_GAMEPAD_X;
                if (rdata[7] > 0x20) pad->wButtons |= XINPUT_GAMEPAD_Y;
                if (rdata[8] > 0x20) pad->wButtons |= XINPUT_GAMEPAD_RIGHT_SHOULDER;
                if (rdata[9] > 0x20) pad->wButtons |= XINPUT_GAMEPAD_LEFT_SHOULDER;

                //Map the left and right triggers
                pad->bLeftTrigger = rdata[10];
                pad->bRightTrigger = rdata[11];

                //Map analog sticks
                pad->sThumbLX = rdata[13] << 8 | rdata[12];
                pad->sThumbLY = rdata[15] << 8 | rdata[14];
                pad->sThumbRX = rdata[17] << 8 | rdata[16];
                pad->sThumbRY = rdata[19] << 8 | rdata[18];

                xid_itf->new_pad_data = true;
            }
        }
        if (xid_itf->new_pad_data)
        {
            tuh_xinput_report_received_cb(dev_addr, instance, xid_itf, sizeof(xinputh_interface_t));
            xid_itf->new_pad_data = false;
        } else {
            tuh_xinput_receive_report(dev_addr, instance);
        }
    }
    else
    {
        if (tuh_xinput_report_sent_cb)
        {
            tuh_xinput_report_sent_cb(dev_addr, instance, xid_itf->epout_buf, xferred_bytes);
        }
    }

    return true;
}

void xinputh_close(uint8_t dev_addr)
{
    TU_VERIFY(dev_addr <= CFG_TUH_DEVICE_MAX, );
    xinputh_device_t *xinput_dev = get_dev(dev_addr);

    for (uint8_t inst = 0; inst < xinput_dev->inst_count; inst++)
    {
        if (tuh_xinput_umount_cb)
        {
            tuh_xinput_umount_cb(dev_addr, inst);
        }
    }
    tu_memclr(xinput_dev, sizeof(xinputh_device_t));
}

#ifndef DRIVER_NAME
#if CFG_TUSB_DEBUG >= CFG_TUH_LOG_LEVEL
  #define DRIVER_NAME(_name)    .name = _name,
#else
  #define DRIVER_NAME(_name)
#endif
#endif

usbh_class_driver_t const usbh_xinput_driver =
{
    DRIVER_NAME("XINPUT")
    .init       = xinputh_init,
    .open       = xinputh_open,
    .set_config = xinputh_set_config,
    .xfer_cb    = xinputh_xfer_cb,
    .close      = xinputh_close
};

#endif
