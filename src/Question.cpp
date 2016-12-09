//
// Created by rdelfin on 12/9/16.
//

#include "learning_helpful_humans/Question.h"
#include <learning_helpful_humans/request/GetFieldValue.h>
#include <random>

static std::default_random_engine generator;

Question::Question(bool random) {
    if(random) {
        GetFieldValue getQuestions("questions/list.json");
        json questions = getQuestions.performAsJson();

        std::uniform_int_distribution<size_t> randInt(0, questions.size() - 1);
        this->id = randInt(generator);
        json question = questions[this->id];

        this->question = question["q"];
        this->prereq = (question["has_prereq"] == true ? question["prereq"] : -1);
    }
}

Question::Question(int id, std::string question, int prereq)
    : id(id), question(question), prereq(prereq) {
}