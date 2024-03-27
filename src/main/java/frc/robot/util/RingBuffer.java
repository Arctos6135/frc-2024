package frc.robot.util;

import java.util.Optional;

public class RingBuffer<T> {
    // the array to store values
    private final T[] buffer;
    // the next empty spot in the array
    private int next = 0;
    // the number of elements that have been written
    private int totalWrites = 0;

    @SuppressWarnings("unchecked")
    public RingBuffer(int size) {
        buffer = (T[]) new Object[size];
    }

    // 0 is the last element written 1, is the second last, etc
    public Optional<T> get(int i) {
        if (i >= totalWrites || i >= buffer.length) {
            return Optional.empty();
        } else {
            return Optional.of(buffer[i % buffer.length]);
        }
    }

    public void add(T element) {
        buffer[next] = element;
        next = (next + 1) % buffer.length;
        totalWrites += 1;
    }
}
