#include <iostream>

using namespace std;

class Queue {
private:
    int* arr;
    int front;
    int rear;
    int capacity;
    int size;

public:
    Queue(int queueSize) {
        front = 0;
        rear = -1;
        size = 0;
        capacity = queueSize;
        arr = new int[capacity];
    }

    void enqueue(int item) {
        if (isFull()) {
            cout << "Queue overflow: Cannot enqueue " << item << ". Queue is full.\n";
            return;
        }

        rear = (rear + 1) % capacity;
        arr[rear] = item;
        ++size;
        cout << "Enqueued " << item << " into the queue.\n";
    }

    void dequeue() {
        if (isEmpty()) {
            cout << "Queue underflow: Cannot dequeue. Queue is empty.\n";
            return;
        }

        int item = arr[front];
        front = (front + 1) % capacity;
        --size;
        cout << "Dequeued " << item << " from the queue.\n";
    }

    int peek() const {
        if (isEmpty()) {
            cout << "Queue is empty.\n";
            return -1; 
        }

        return arr[front];
    }

    bool isEmpty() const {
        return size == 0;
    }

    bool isFull() const {
        return size == capacity;
    }

    void display() const {
        if (isEmpty()) {
            cout << "Queue is empty.\n";
            return;
        }

        cout << "Queue contents: ";
        int index = front;
        for (int i = 0; i < size; ++i) {
            cout << arr[index] << " ";
            index = (index + 1) % capacity;
        }
        cout << "\n";
    }
};