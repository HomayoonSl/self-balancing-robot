<div dir="rtl">

<div dir="rtl">

> **شفافیت توسعه:** کدهای این مخزن با کمک مدل‌های زبانی Gemini 3.8، DeepSeek و Claude 5.5 Sonnet توسعه داده شده‌اند. فقط از دسترسی/پلن رایگان استفاده شده و هیچ اشتراک یا API پولی به کار نرفته است. همهٔ کدها بازبینی شده و روی سخت‌افزار واقعی با موفقیت تست و پیاده‌سازی شده‌اند. ویدیوی نمایش پروژه بر پایهٔ کدهای کامیت [`eb74cd6`](https://github.com/HomayoonSl/self-balancing-robot/commit/eb74cd6349497c950bc9238eed6e0f363d144567) است: [تماشای ویدیو](https://homayoon.net/wp-content/uploads/2026/10/Self-Balancing-Robot_20261023.mp4). ویدیوهای بعدی بر اساس کدهای کامل‌تر به‌زودی منتشر می‌شوند؛ با این حال اطمینان داده می‌شود که همهٔ پیاده‌سازی‌ها با موفقیت انجام و تست شده‌اند.

</div>

# ربات خودمتعادل‌کننده

این پروژه پیاده‌سازی یک ربات خودمتعادل‌کننده بر پایه مدل فیزیکی آونگ معکوس است که با استفاده از میکروکنترلر STM32 و زبان برنامه‌نویسی Rust توسعه یافته است. هدف این سیستم حفظ تعادل پایدار روی دو چرخ با بهره‌گیری از کنترل دقیق موتورها و لنگر مکانیکی است.

## مشخصات سخت‌افزاری
* **واحد پردازش:** میکروکنترلر STM32F103 (برد Blue Pill)
* **محرک‌ها:** موتورهای گیربکس‌دار TT با درایور DRV8833
* **سنسورها:** ماژول IMU مدل BMI160 و انکودر مغناطیسی AS5600
* **مکانیزم تعادل:** لنگر مکانیکی شامل میله و گوی برای تنظیم ممان اینرسی
* **تغذیه:** باتری لیتیوم-یون 2S همراه با رگولاتور کاهنده و مدار نظارت بر ولتاژ

## نرم‌افزار
* توسعه‌یافته به زبان **Rust** در محیط بدون سیستم‌عامل (`no_std`)
* استفاده از لایه انتزاعی سخت‌افزار برای مدیریت پین‌ها، تایمرهای PWM و ارتباطات I2C مجزا

</div>

---

> **Development disclosure:** The code in this repository was developed with assistance from Gemini 3.8, DeepSeek, and Claude 5.5 Sonnet language models. Only free-tier access was used; no paid subscription or paid API access was used. All code was reviewed and successfully tested and deployed on real hardware. The demo video is based on the code at commit [`eb74cd6`](https://github.com/HomayoonSl/self-balancing-robot/commit/eb74cd6349497c950bc9238eed6e0f363d144567): [Watch the video](https://homayoon.net/wp-content/uploads/2026/10/Self-Balancing-Robot_20261023.mp4). Further videos based on more complete code will be published soon; nevertheless, all implementations have been successfully completed and tested.


# Self-Balancing Robot

This project implements a self-balancing robot based on the inverted pendulum physics model, developed using an STM32 microcontroller and the Rust programming language. The goal is to maintain stable balance on two wheels through precise motor control and a mechanical inertia rod.

## Hardware Specifications
* **Microcontroller:** STM32F103 (Blue Pill board)
* **Actuators:** TT gear motors driven by DRV8833
* **Sensors:** BMI160 IMU module and AS5600 magnetic encoder
* **Balance Mechanism:** Mechanical rod and sphere configuration for moment of inertia tuning
* **Power:** 2S Li-ion battery system with a buck regulator and voltage monitoring

## Software
* Developed in **Rust** for bare-metal (`no_std`) environments
* Utilizes hardware abstraction layers for managing pins, PWM timers, and dedicated I2C buses
