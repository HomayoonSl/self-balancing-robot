<div dir="rtl">


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
