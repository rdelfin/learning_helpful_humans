//
// Created by rdelfin on 12/9/16.
//

#ifndef PROJECT_QUESTION_HPP
#define PROJECT_QUESTION_HPP


struct Question {
public:
    Question(bool random = false);
    Question(int id, std::string question, int prereq = -1);

    ~Question() { }

    int id;
    std::string question;
    int prereq;
};


#endif //PROJECT_QUESTION_HPP
