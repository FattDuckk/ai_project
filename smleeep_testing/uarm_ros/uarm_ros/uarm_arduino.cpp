#define F_CPU 16000000UL
#include <avr/io.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <chrono>
#include <thread>

// UART Init
void uart_init(uint16_t baud) {
    uint16_t ubrr = F_CPU / 16 / baud - 1;
    UBRR0H = (ubrr >> 8);
    UBRR0L = ubrr;
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);  // Enable TX and RX
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);  // 8-bit data
}

void uart_send(char c) {
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = c;
}

void uart_send_string(const char* str) {
    while (*str) uart_send(*str++);
}

char uart_receive() {
    while (!(UCSR0A & (1 << RXC0)));
    return UDR0;
}

// Servo PWM using Timer1 (16-bit, on PB1/OC1A)
void servo_init() {
    DDRB |= (1 << PB1); // Output on OC1A (Pin 9)

    TCCR1A = (1 << COM1A1) | (1 << WGM11);
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);  // Prescaler = 8
    ICR1 = 40000; // 50Hz PWM (20ms)
}

// Set pulse width in microseconds (1000-2000us for servo)
void servo_set_angle(uint16_t us) {
    OCR1A = us * 2; // Scale to 16MHz/8
}

// Basic command parser
void process_command(char* cmd) {
    if (strncmp(cmd, "MOVE", 4) == 0) {
        int angle = atoi(cmd + 5);  // crude parsing: "MOVE 150"
        uint16_t us = (angle * 11) + 1000; // Map 0–180° to 1000–2000us
        if (us > 2000) us = 2000;
        if (us < 1000) us = 1000;
        servo_set_angle(us);
        uart_send_string("Moved servo\n");
    }
    else if (strcmp(cmd, "SUCTION ON") == 0) {
        PORTD |= (1 << PD7);  // Turn on pump (digital high)
        uart_send_string("Pump ON\n");
    }
    else if (strcmp(cmd, "SUCTION OFF") == 0) {
        PORTD &= ~(1 << PD7);  // Turn off pump
        uart_send_string("Pump OFF\n");
    }
    else {
        uart_send_string("Unknown cmd\n");
    }
}

void _delay_ms(int ms) {
  // Not accurate, just placeholder for simulation
  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

int main() {
    char buffer[32];
    uint8_t i = 0;

    uart_init(9600);
    servo_init();
    DDRD |= (1 << PD7);  // Pump control on PD7

    uart_send_string("uArm ready\n");

    while (1) {
        char c = uart_receive();
        if (c == '\n' || c == '\r') {
            buffer[i] = '\0';
            process_command(buffer);
            i = 0;
        } else {
            if (i < sizeof(buffer) - 1) {
                buffer[i++] = c;
            }
        }
    }
}