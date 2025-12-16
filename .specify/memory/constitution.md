<!--
Sync Impact Report:
- Version change: 1.0.0 → 1.0.0 (initial creation)
- Modified principles: None (new document)
- Added sections: All sections added as initial constitution
- Removed sections: None
- Templates requiring updates: N/A (initial creation)
- Follow-up TODOs: None
-->

# AI/Spec-Driven Book with Embedded RAG Chatbot Constitution

## Core Principles

### I. Spec-First Development
Every feature and component must be defined in a specification before implementation begins. Specifications must be comprehensive, testable, and validated through the Spec-Kit Plus framework before any code is written. This ensures alignment between business requirements and technical implementation.

### II. Accuracy and Documentation Alignment
All implementations must strictly align with official documentation and established APIs. No hallucinated APIs, undocumented features, or assumptions about third-party services are permitted. All code examples must be executable and well-commented to serve as reliable reference material.

### III. Content Integrity (NON-NEGOTIABLE)
All book content must be 100% original with zero plagiarism. Content must be instructional and developer-focused, adhering to Docusaurus-compatible MDX standards. Every explanation must be grounded in primary sources and verified through authoritative documentation.

### IV. RAG System Fidelity
The RAG chatbot must provide answers strictly grounded in the book content only. Responses must be factually accurate and cite specific sections of the book content. The system must not generate hallucinated information or venture beyond the provided knowledge base.

### V. Free-Tier Infrastructure Compatibility
All technology choices and architectural decisions must be compatible with free-tier services wherever possible. This includes Qdrant Cloud Free Tier for vector storage, Neon Serverless Postgres for metadata, and GitHub Pages for deployment. Resource usage must be optimized for cost-effectiveness.

### VI. Security-First Credential Handling
All sensitive information, API keys, and credentials must be handled through environment variables or secure vault systems. No hardcoded credentials, secrets, or tokens are permitted in the codebase. Secure credential handling is non-negotiable for all deployments.

## Technical Standards

### Content and Code Standards
- Content must be written in Docusaurus-compatible MDX format
- Claude Code must be used to generate and validate book content and code
- All code examples must be executable, well-commented, and based on primary sources
- RAG chatbot must use FastAPI backend with OpenAI Agents/ChatKit SDKs
- Vector storage must utilize Qdrant Cloud (Free Tier)
- Metadata storage must use Neon Serverless Postgres

### Quality Assurance
- All code must be traceable between specs, content, and implementation
- Executable code examples must be provided for every concept
- No undocumented APIs or features may be used
- All explanations must be grounded in primary sources

## Development Workflow

### Implementation Process
- Follow Spec-Kit Plus methodology: spec → plan → tasks → implementation
- Use Claude Code for all development tasks and code generation
- Maintain strict separation between business understanding and technical implementation
- Create Prompt History Records (PHRs) for all significant development activities
- Generate Architectural Decision Records (ADRs) for significant technical decisions

### Review and Validation
- All specifications must undergo validation before implementation
- Code reviews must verify compliance with constitutional principles
- Content accuracy must be validated against primary sources
- RAG functionality must be tested with sample queries from book content
- Deployment must be validated on GitHub Pages

## Governance

This constitution governs all aspects of the AI/Spec-Driven Book with Embedded RAG Chatbot project. All development activities, architectural decisions, and implementation choices must comply with these principles. Amendments to this constitution require explicit documentation of changes, justification for deviations, and approval from project stakeholders. All team members are responsible for ensuring compliance with these principles during code reviews, testing, and deployment activities.

**Version**: 1.0.0 | **Ratified**: 2025-12-16 | **Last Amended**: 2025-12-16
