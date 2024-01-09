#include <iostream>

using namespace std;

class Stack {
private:
    int* arr;
    int top;
    int capacity;

public:
    Stack(int size){
		top = -1;
		capacity = size;
        arr = new int[capacity];
    }

    void push(int item) {
        if (isFull()) {
            cout << "Stack overflow: Cannot push " << item << ". Stack is full.\n";
            return;
        }

        arr[++top] = item;
        cout << "Pushed " << item << " onto the stack.\n";
    }

    void pop() {
        if (isEmpty()) {
            cout << "Stack underflow: Cannot pop. Stack is empty.\n";
            return;
        }

        --top;
        cout << "Popped an element from the stack.\n";
    }

    void peek() {
        if (isEmpty()) {
            cout << "Stack is empty.\n";
            return;
        }

        cout << "Top element: " << arr[top] << "\n";
    }

    bool isEmpty() {
        return top == -1;
    }

    bool isFull() {
        return top == capacity - 1;
    }

    void clear() {
        top = -1;
        cout << "Stack cleared.\n";
    }

    void display() {
        cout << "Stack contents: ";
        for (int i = top; i >= 0; --i) {
            cout << arr[i] << " ";
        }
        cout << "\n";
    }
};