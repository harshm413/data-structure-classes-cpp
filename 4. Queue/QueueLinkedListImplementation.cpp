#include <iostream>

using namespace std;

class Queue {
private:
    class Node {
    public:
        int data;
        Node* next;

        Node(int value) {
			data = value;
			next = nullptr;
		}
    };

    Node* front;
    Node* rear;

public:
    Queue() {
        front = nullptr;
        rear = nullptr;
    }

    void enqueue(int item) {
        Node* newNode = new Node(item);

        if (isEmpty()) {
            front = rear = newNode;
        } else {
            rear->next = newNode;
            rear = newNode;
        }

        cout << "Enqueued " << item << " into the queue.\n";
    }

    void dequeue() {
        if (isEmpty()) {
            cout << "Queue underflow: Cannot dequeue. Queue is empty.\n";
            return;
        }

        Node* temp = front;
        int item = temp->data;

        if (front == rear) {
            front = rear = nullptr;
        } else {
            front = front->next;
        }

        delete temp;
        cout << "Dequeued " << item << " from the queue.\n";
    }

    int peek() {
        if (isEmpty()) {
            cout << "Queue is empty.\n";
            return -1; 
        }

        return front->data;
    }

    bool isEmpty() {
        return front == nullptr;
    }

    void clear() {
        while (!isEmpty()) {
            dequeue();
        }
    }

    void display() {
        if (isEmpty()) {
            cout << "Queue is empty.\n";
            return;
        }

        cout << "Queue contents: ";
        Node* current = front;
        while (current != nullptr) {
            cout << current->data << " ";
            current = current->next;
        }
        cout << "\n";
    }
};