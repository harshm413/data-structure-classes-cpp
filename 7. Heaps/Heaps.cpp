#include <iostream>
#include <vector>
using namespace std;

class Heap {
private:
    vector<int> items;
    int size;
    int capacity;

public:
    Heap(int capacity) : size(0), capacity(capacity) {
        items.resize(capacity);
    }

    void insert(int value) {
        if (isFull()) {
            cout << "Heap is full. Cannot insert " << value << endl;
            return;
        }

        items[size++] = value;
        bubbleUp();
    }

    int remove() {
        if (isEmpty()) {
            cout << "Heap is empty. Cannot remove." << endl;
            return -1;
        }

        int root = items[0];
        items[0] = items[--size];
        bubbleDown();

        return root;
    }

    bool isEmpty() {
        return size == 0;
    }

    int max() {
        if (isEmpty()) {
            cout << "Heap is empty. No maximum value." << endl;
            return -1;
        }

        return items[0];
    }

private:
    bool isFull() {
        return size == capacity;
    }

    void bubbleUp() {
        int index = size - 1;
        while (index > 0 && items[index] > items[parent(index)]) {
            swap(index, parent(index));
            index = parent(index);
        }
    }

    void bubbleDown() {
        int index = 0;
        while (index < size && !isValidParent(index)) {
            int largerChildIndex = getLargerChildIndex(index);
            swap(index, largerChildIndex);
            index = largerChildIndex;
        }
    }

    int parent(int index) {
        return (index - 1) / 2;
    }

    int getLeftChildIndex(int index) {
        return 2 * index + 1;
    }

    int getRightChildIndex(int index) {
        return 2 * index + 2;
    }

    int getLargerChildIndex(int index) {
        if (!hasLeftChild(index))
            return index;
        if (!hasRightChild(index))
            return getLeftChildIndex(index);
        return (leftChild(index) > rightChild(index)) ? getLeftChildIndex(index) : getRightChildIndex(index);
    }

    bool hasLeftChild(int index) {
        return getLeftChildIndex(index) < size;
    }

    bool hasRightChild(int index) {
        return getRightChildIndex(index) < size;
    }

    bool isValidParent(int index) {
        if (!hasLeftChild(index))
            return true;
        bool isValid = items[index] >= leftChild(index);
        if (hasRightChild(index))
            isValid &= items[index] >= rightChild(index);
        return isValid;
    }

    int leftChild(int index) {
        return items[getLeftChildIndex(index)];
    }

    int rightChild(int index) {
        return items[getRightChildIndex(index)];
    }

    void swap(int first, int second) {
        int temp = items[first];
        items[first] = items[second];
        items[second] = temp;
    }
};
