from telegram import Bot
BOT_TOKEN = "DEIN_BOT_TOKEN"
bot = Bot(token=BOT_TOKEN)
bot.get_me()  # sollte Bot-Infos zurückgeben
