//
// Created by anaple on 2024/4/25.
//

#include "queue.h"


bool CircularQueue::enqueue(std::string data) {
    if (isFull) {
        return false;
    }
    buffer[tail] = data;
    tail = (tail + 1) % maxSize;
    isFull = tail == head;
    return true;
}
bool CircularQueue::dequeue(std::string &data) {
    if (isEmpty()) {
        return false;
    }
    data = buffer[head];
    head = (head + 1) % maxSize;
    isFull = false;
    return true;
}

bool CircularQueue::isEmpty() const{
    return head == tail && !isFull;
}

int CircularQueue::getSize() const{
    return (tail - head + maxSize) % maxSize;
}
