#pragma once
#include <iostream>
using namespace std;
  
// Node class to represent
// a node of the linked list.
template <typename T> class Node {
public:
    T data;
    Node* next;
  
    // Default constructor
    Node()
    {
        data = 0;
        next = NULL;
    }
  
    // Parameterised Constructor
    Node(T data)
    {
        this->data = data;
        this->next = NULL;
    }
};
  
// Linked list class to
// implement a linked list.
template <typename T> class Linkedlist {
    Node<T>* head;
  
public:
    int length=0;
    // Default constructor
    Linkedlist() { head = NULL; }
  
    // Function to insert a
    // node at the end of the
    // linked list.
    void insertLast(T);

    //function to insert a node at the beginning of the linked list
    void insertFirst(T);
  
    // Function to print the
    // linked list.
    void printList();
  
    // Function to delete the
    // node at given position
    void deleteNode(int);

    Node<T>* get(int);
};
//0 indexed
template <typename T> Node<T>* Linkedlist<T>::get(int index){
    if(length==0){
        return nullptr;
    }
    Node<T>* current = head;
    for(int i=0; i<index; i++){
        current = current->next;
    }
    return current;
}
// Function to delete the
// node at given position
template <typename T> void Linkedlist<T>::deleteNode(int nodeOffset)
{
    Node<T> *temp1 = head, *temp2 = NULL;
    int ListLen = 0;
  
    if (head == NULL) {
        cout << "List empty." << endl;
        return;
    }
  
    // Find length of the linked-list.
    while (temp1 != NULL) {
        temp1 = temp1->next;
        ListLen++;
    }
  
    // Check if the position to be
    // deleted is greater than the length
    // of the linked list.
    if (ListLen < nodeOffset) {
        cout << "Index out of range"
             << endl;
        return;
    }
  
    // Declare temp1
    temp1 = head;
  
    // Deleting the head.
    if (nodeOffset == 1) {
  
        // Update head
        head = head->next;
        delete temp1;
        return;
    }
  
    // Traverse the list to
    // find the node to be deleted.
    while (nodeOffset-- > 1) {
  
        // Update temp2
        temp2 = temp1;
  
        // Update temp1
        temp1 = temp1->next;
    }
  
    // Change the next pointer
    // of the previous node.
    temp2->next = temp1->next;
  
    // Delete the node
    delete temp1;
    length--;
}
  
// Function to insert a new node.
template <typename T> void Linkedlist<T>::insertLast(T data)
{
    // Create the new Node.
    Node<T>* newNode = new Node<T>(data);
  
    // Assign to head
    if (head == NULL) {
        head = newNode;
        return;
    }
  
    // Traverse till end of list
    Node<T>* temp = head;
    while (temp->next != NULL) {
  
        // Update temp
        temp = temp->next;
    }
  
    // Insert at the last.
    temp->next = newNode;
    length++;
}
template <typename T> void Linkedlist<T>::insertFirst(T data)
{
    // Create the new Node.
    Node<T>* newNode = new Node<T>(data);
  
    // Assign to head
    if (head == NULL) {
        head = newNode;
        return;
    }
    //save head
    Node<T>* temp = head;
    //insert
    head = newNode;
    //assign next
    head->next=temp;
    length++;
}
// Function to print the
// nodes of the linked list.
template <typename T> void Linkedlist<T>::printList()
{
    Node<T>* temp = head;
  
    // Check for empty list.
    if (head == NULL) {
        cout << "List empty" << endl;
        return;
    }
  
    // Traverse the list.
    while (temp != NULL) {
        cout << temp->data << " ";
        temp = temp->next;
    }
}