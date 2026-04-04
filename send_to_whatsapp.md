# WhatsApp Business API - Send Messages Guide

## Account Details

| Field | Value |
|---|---|
| **Business Name** | Yasalk |
| **App Name** | n8n test |
| **App ID** | `724019983707645` |
| **WABA ID** | `1974463720159045` |
| **Phone Number ID** | `1028757036983681` |
| **Sender Number** | `+971 50 132 7129` |
| **Business ID** | `5597348943643216` |

## Access Token (System User)

```
EAAKSfgnwbf0BQZCqpn6H6WbbGZAWoXqe92BvjwUbq08jZAEfsfY6cgR8OnG83JQG2ZAs5QZA4N0ZCf9bA3ev69ixt5EP2TwnarHVAekOXqHskZBFFutZBZBlETKZCCZB4RpDBQHZBeT6Leo9URjBTmuZBMjKKQFDSb646IbXjv9LOdIvWa5o4rv0xoElAVmZByjSI0RHbqyczhCTf5ZCNNe74EwOdEGViNbXRyV5m6ZB2PWBBHrR
```

**Token type:** System User
**Permissions:** `whatsapp_business_messaging`, `whatsapp_business_management`, `whatsapp_business_manage_events`, `public_profile`
**Expires at:** 1776963667 (Unix timestamp)

> **Note:** If the token has expired, generate a new one from Business Settings > System Users > Generate Token. Make sure the system user has the app "n8n test" assigned with Full Control, and select all WhatsApp permissions when generating.

---

## API Base URL

```
https://graph.facebook.com/v22.0
```

---

## 1. Send a Plain Text Message

> **Requirement:** The recipient must have messaged the business number (`+971 50 132 7129`) within the last 24 hours. This opens a conversation window.

### Using curl

```bash
curl -s -X POST \
  'https://graph.facebook.com/v22.0/1028757036983681/messages' \
  -H 'Authorization: Bearer ACCESS_TOKEN' \
  -H 'Content-Type: application/json' \
  -d '{
    "messaging_product": "whatsapp",
    "to": "RECIPIENT_PHONE_NUMBER",
    "type": "text",
    "text": { "body": "Your message here" }
  }'
```

### Using Python (recommended — avoids JSON encoding issues with curl on some shells)

```python
import urllib.request, json

url = 'https://graph.facebook.com/v22.0/1028757036983681/messages'
data = json.dumps({
    'messaging_product': 'whatsapp',
    'to': 'RECIPIENT_PHONE_NUMBER',
    'type': 'text',
    'text': {'body': 'Your message here'}
}).encode('utf-8')

req = urllib.request.Request(url, data=data, method='POST')
req.add_header('Authorization', 'Bearer ACCESS_TOKEN')
req.add_header('Content-Type', 'application/json')

with urllib.request.urlopen(req) as resp:
    print(resp.read().decode())
```

### Phone Number Format

- Use international format **without** the `+` sign
- Example: `971508996646` (UAE number +971 50 899 6646)

### Successful Response

```json
{
  "messaging_product": "whatsapp",
  "contacts": [{"input": "971508996646", "wa_id": "971508996646"}],
  "messages": [{"id": "wamid.XXXXXXXXXXXX"}]
}
```

---

## 2. Send a Template Message

> Template messages can be sent **anytime** without the 24-hour window. The template must be **APPROVED** by Meta first.

```bash
curl -s -X POST \
  'https://graph.facebook.com/v22.0/1028757036983681/messages' \
  -H 'Authorization: Bearer ACCESS_TOKEN' \
  -H 'Content-Type: application/json' \
  -d '{
    "messaging_product": "whatsapp",
    "to": "RECIPIENT_PHONE_NUMBER",
    "type": "template",
    "template": {
      "name": "TEMPLATE_NAME",
      "language": { "code": "en_US" },
      "components": [
        {
          "type": "header",
          "parameters": [
            { "type": "text", "text": "header_variable_value" }
          ]
        },
        {
          "type": "body",
          "parameters": [
            { "type": "text", "text": "body_variable_1" },
            { "type": "text", "text": "body_variable_2" }
          ]
        }
      ]
    }
  }'
```

---

## 3. Create a New Template

```bash
curl -s -X POST \
  'https://graph.facebook.com/v22.0/1974463720159045/message_templates' \
  -H 'Authorization: Bearer ACCESS_TOKEN' \
  -H 'Content-Type: application/json' \
  -d '{
    "name": "template_name_lowercase_underscores",
    "language": "en_US",
    "category": "UTILITY",
    "components": [
      {
        "type": "HEADER",
        "format": "TEXT",
        "text": "Header with variable {{1}}",
        "example": { "header_text": ["Example Value"] }
      },
      {
        "type": "BODY",
        "text": "Body text with {{1}} and {{2}} variables.",
        "example": { "body_text": [["Example1", "Example2"]] }
      },
      {
        "type": "FOOTER",
        "text": "Footer text here"
      }
    ]
  }'
```

### Template Tips for Approval

- Use **UTILITY** category for transactional messages (order updates, receipts, etc.)
- Include **variables** (`{{1}}`, `{{2}}`) with realistic **examples**
- Make the body **long and descriptive** to give Meta's review bot more context
- Make it look **transactional** (order updates, invoices, confirmations)
- Always provide the `example` field with sample values
- `hello_world` template only works with Meta's public test numbers, not your own

### Check Template Status

```bash
curl -s 'https://graph.facebook.com/v22.0/1974463720159045/message_templates?name=TEMPLATE_NAME&access_token=ACCESS_TOKEN'
```

Status will be: `PENDING`, `APPROVED`, or `REJECTED`.

### Delete a Template

```bash
curl -s -X DELETE \
  'https://graph.facebook.com/v22.0/1974463720159045/message_templates?name=TEMPLATE_NAME' \
  -H 'Authorization: Bearer ACCESS_TOKEN'
```

---

## 4. List All Templates

```bash
curl -s 'https://graph.facebook.com/v22.0/1974463720159045/message_templates?access_token=ACCESS_TOKEN'
```

---

## Current Templates

### `order_status_update` (Status: PENDING)

- **Category:** UTILITY
- **Header:** `Order Update - #{{1}}`
- **Body:** `Dear {{1}}, your order #{{2}} placed on {{3}} has been updated. The current status of your order is: {{4}}. The estimated delivery date is {{5}}. If you have any questions regarding your order or need further assistance, please do not hesitate to contact our support team. Thank you for choosing Yasalk.`
- **Footer:** `Yasalk - Order Management System`
- **Variables:** header(1: order number), body(1: customer name, 2: order number, 3: order date, 4: status, 5: delivery date)

#### Example: Send `order_status_update`

```json
{
  "messaging_product": "whatsapp",
  "to": "971508996646",
  "type": "template",
  "template": {
    "name": "order_status_update",
    "language": { "code": "en_US" },
    "components": [
      {
        "type": "header",
        "parameters": [
          { "type": "text", "text": "10234" }
        ]
      },
      {
        "type": "body",
        "parameters": [
          { "type": "text", "text": "Ahmed" },
          { "type": "text", "text": "10234" },
          { "type": "text", "text": "February 20, 2026" },
          { "type": "text", "text": "Shipped" },
          { "type": "text", "text": "February 25, 2026" }
        ]
      }
    ]
  }
}
```

---

## 5. Verify Token

```bash
curl 'https://graph.facebook.com/v22.0/me?access_token=ACCESS_TOKEN'
```

## 6. Debug Token (check permissions & expiry)

```bash
curl 'https://graph.facebook.com/v22.0/debug_token?input_token=ACCESS_TOKEN&access_token=ACCESS_TOKEN'
```

## 7. Register Phone Number (if needed)

If you get `(#133010) Account not registered` errors, the sender phone number needs to be registered:

```bash
curl -s -X POST \
  'https://graph.facebook.com/v22.0/1028757036983681/register' \
  -H 'Authorization: Bearer ACCESS_TOKEN' \
  -H 'Content-Type: application/json' \
  -d '{ "messaging_product": "whatsapp", "pin": "123456" }'
```

---

## Common Errors

| Error | Meaning | Fix |
|---|---|---|
| `(#133010) Account not registered` | Sender phone not registered with Cloud API | Run the register endpoint (section 7) |
| `(#131058) Hello World templates can only be sent from Public Test Numbers` | `hello_world` is Meta's test-only template | Create and use your own template |
| `(#100) The parameter messaging_product is required` | JSON body not parsed correctly | Use Python method or ensure proper Content-Type header |
| `(#100) Missing Permission` | Token lacks required permissions | Regenerate token with WhatsApp permissions |
| `Object does not exist` | Wrong Phone Number ID or WABA ID | Verify IDs from Business Manager |

---

## Useful Links

- **Meta Developer App:** https://developers.facebook.com/apps/724019983707645/
- **WhatsApp API Setup:** https://developers.facebook.com/apps/724019983707645/whatsapp-business/wa-dev-console/
- **Business Settings:** https://business.facebook.com/settings/
- **WhatsApp Manager:** https://business.facebook.com/latest/whatsapp_manager/overview/?business_id=5597348943643216&asset_id=1974463720159045
- **Message Templates:** https://business.facebook.com/wa/manage/message-templates/
- **Graph API Docs:** https://developers.facebook.com/docs/whatsapp/cloud-api/
