# Language

This folder provides sentence processing and utilities through LLM model inference on ollama

## Packages

- lasr_llm: LLM integration package that exposes model-backed services for language tasks.
- lasr_llm_interfaces: Shared message and service interface definitions for the language stack.
 
## Overview

The language stack is used to process transcripts, extract information, and support higher-level decision making for the robot. The lasr_llm package in particular provides LLM-based services for tasks such as:
- transcript processing
- information extraction
- structured parsing of user input
- task-specific language understanding