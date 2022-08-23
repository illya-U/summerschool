import logging
from aiogram import Bot, Dispatcher, executor, types
from os import getenv
from sys import exit
from aiogram.types import ReplyKeyboardMarkup, ReplyKeyboardRemove, KeyboardButton
from Uart_taker import temperature_UART, presure_UART, All_UART
from aiogram.utils.exceptions import BotBlocked

bot_token = getenv("BOT_TOKEN")
if not bot_token:
    exit("Error: no token provided")
bot = Bot(token = bot_token)
dp = Dispatcher(bot)
logging.basicConfig(level = logging.INFO)

keyboard = types.ReplyKeyboardMarkup(resize_keyboard=True)
button1 = KeyboardButton("Presure")
button2 = KeyboardButton("Temperature")
button3 = KeyboardButton("All")
keyboard.add(button1).add(button2).add(button3)

@dp.message_handler(commands = "start")
async def starting(message: types.Message):
    await message.answer("Hi " + message.from_user.first_name, reply_markup = keyboard)

@dp.message_handler(lambda message:message.text == "Presure")
async def take_presure(message: types.Message):
    #take presure
        await message.answer('Presure = ' + presure_UART())
    #
@dp.message_handler(lambda message:message.text == "Temperature")
async def take_temperature(message: types.Message):
    #take temperature
        await message.answer('Temperature = '+ temperature_UART())
    #
@dp.message_handler(lambda message:message.text == "All")
async def take_All(message: types.Message):
    #take all
        list = All_UART()
        await message.answer("Presure = " + list[1])
        await message.answer("Temperature = " + list[0])
    #
@dp.errors_handler(exception = BotBlocked)
async def error_bot_blocked(update: types.Update, exception: BotBlocked):
        print(f"Me blocked by user!\nMessage: {update}\nError: {exception}")
        return True

if __name__ == "__main__":
    executor.start_polling(dp, skip_updates = True)
