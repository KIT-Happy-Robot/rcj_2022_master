# send_gmail

# Overview
指定したGmailアカウントから、宛名、件名、本文を指定して送信するサービスサーバー

# Description
### Send Gmail
| Communication | Name | Type | Request | Result |
| ---- | ---- | ---- | ---- | ---- |
| Service | /send_gmail_server | [SendGmail](https://github.com/HappyYusuke/send_gmail/blob/main/srv/SendGmail.srv) | string型: `from_address` <br>string型: `password` <br>string[]型: `to_address` <br>string型: `mail_subject` <br>string型: `body_message` | bool型: `result` |
### Request
| Request | Contents |
| ---- | ---- |
| string型: `from_address` | 送信元のGmailのメールアドレス |
| string型: `password` | 送信元のGmailアカウントのアプリパスワード <br>:warning:**アプリパスワードの取得方法は[こちら](https://support.google.com/accounts/answer/185833?hl=ja)** |
