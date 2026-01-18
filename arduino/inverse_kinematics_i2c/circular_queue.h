#pragma once

#include <stddef.h>

/**
 * Fixed-size circular queue for waypoint management.
 * Template parameter T should be the data type to store (e.g., waypoint struct).
 */
template <typename T>
class CircularQueue {
public:
	static constexpr size_t SIZE = 4;  // Maximum number of elements

	CircularQueue() {
		clear();
	};

	// Check if queue is empty
	bool isEmpty() const {
		return front_ == rear_;
	}

	// Check if queue is full
	bool isFull() const {
		return ((rear_ + 1) % ARRAY_SIZE) == front_;
	}

	// Get front element without removing it
	void front(T& d) {
		d = data[front_];
	}

	// Remove front element
	void pop() {
		front_ = (front_ + 1) % ARRAY_SIZE;
	}

	// Add element to rear
	void push(const T& d) {
		data[rear_] = d;
		rear_ = (rear_ + 1) % ARRAY_SIZE;
	}

	// Clear all elements
	void clear() {
		front_ = rear_ = 0;
	}

private:
	int rear_;
	int front_;
	static constexpr size_t ARRAY_SIZE = (SIZE + 1);  // Extra slot to distinguish full vs empty
	T data[ARRAY_SIZE];
};
